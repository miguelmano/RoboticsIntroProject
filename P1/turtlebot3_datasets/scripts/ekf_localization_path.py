#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class EKFPathVisualizer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ekf_localization_path', anonymous=True)

        # Subscribe to the EKF's filtered odometry data
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)

        # Publisher for the path
        self.path_pub = rospy.Publisher('/ekf_path', Path, queue_size=10)

        # Initialize the path message
        self.path = Path()
        # Frame where the robot is localized
        self.path.header.frame_id = "map"  # EKF is usually in the "map" frame

    def odom_callback(self, msg):
        # Create a PoseStamped message from the filtered odometry data
        pose = PoseStamped()
        pose.header = msg.header  # Use the same timestamp and frame_id
        pose.pose = msg.pose.pose  # Extract the pose information

        # Append the pose to the path
        self.path.poses.append(pose)  # Corrected typo: 'ppose' to 'pose'

        # Make sure to keep updating the header timestamp
        self.path.header.stamp = rospy.Time.now()

        # Publish the path with adjusted position
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        EKFPathVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

