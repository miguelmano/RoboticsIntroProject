#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class AmclPath:
    def __init__(self):
        rospy.init_node('amcl_path_node', anonymous=True)

        # Publisher for the AMCL Path
        self.path_pub_amcl = rospy.Publisher('/amcl_path', Path, queue_size=10)

        # Initialize the Path message
        self.path_amcl = Path()
        self.path_amcl.header.frame_id = "map"

        # Subscriber to the /amcl_pose topic
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

    def amcl_callback(self, msg):
        #rospy.loginfo("AMCL callback triggered")

        # Create a PoseStamped message from the /amcl_pose topic
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append the PoseStamped to the AMCL path
        self.path_amcl.poses.append(pose)

        # Update the header timestamp for the path
        self.path_amcl.header.stamp = rospy.Time.now()

        # Publish the accumulated AMCL path
        self.path_pub_amcl.publish(self.path_amcl)
        #rospy.loginfo("Published AMCL path with %d poses", len(self.path_amcl.poses))

if __name__ == '__main__':
    try:
        AmclPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

