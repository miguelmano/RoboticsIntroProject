#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TFToPath:
    def __init__(self):
        rospy.init_node('tf_to_path', anonymous=True)

        # Publisher for the Ground-Truth Path
        self.path_pub = rospy.Publisher('/ground_truth_path', Path, queue_size=10)

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "map"  # Assuming ground-truth is in the map frame

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Set the transform lookup rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def run(self):
        while not rospy.is_shutdown():
            try:
                # Get the transform between map and mocap_laser_link (ground-truth from mocap)
                (trans, rot) = self.tf_listener.lookupTransform('/map', '/mocap_laser_link', rospy.Time(0))

                # Create a PoseStamped message for the ground-truth pose
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                
                # Set the position (translation)
                pose.pose.position.x = trans[0]
                pose.pose.position.y = trans[1]
                pose.pose.position.z = trans[2]

                # Set the orientation (rotation)
                pose.pose.orientation.x = rot[0]
                pose.pose.orientation.y = rot[1]
                pose.pose.orientation.z = rot[2]
                pose.pose.orientation.w = rot[3]

                # Append the PoseStamped to the path
                self.path.poses.append(pose)

                # Update the header timestamp for the path
                self.path.header.stamp = rospy.Time.now()

                # Publish the accumulated path
                self.path_pub.publish(self.path)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            # Sleep to maintain the desired rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        tf_to_path = TFToPath()
        tf_to_path.run()
    except rospy.ROSInterruptException:
        pass

