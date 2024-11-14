#!/usr/bin/env python

import rospy
from robot_localization.srv import SetPose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path

class EKFLocalization:
    def __init__(self):
        rospy.init_node('ekf_localization_node', anonymous=True)

        # Publisher for the EKF path
        self.path_pub = rospy.Publisher('/ekf_path', Path, queue_size=10)

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = "map"  # Set to the correct frame

        # Subscriber for the EKF pose
        self.ekf_pose_sub = rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.ekf_pose_callback)

        # Subscribers for odometry and IMU data (if needed for additional processing)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)

        # Set the initial pose
        self.set_initial_pose()

    def ekf_pose_callback(self, msg):
        # Create a PoseStamped message from the EKF pose
        pose = PoseStamped()
        pose.header = msg.header  # Use the header from the EKF pose
        pose.pose = msg.pose.pose  # Extract the pose

        # Append the PoseStamped to the path
        self.path.poses.append(pose)

        # Update the header timestamp for the path
        self.path.header.stamp = rospy.Time.now()

        # Publish the accumulated path
        self.path_pub.publish(self.path)
        rospy.loginfo("EKF path published.")

    def odom_callback(self, msg):
        rospy.loginfo("Odometry data received.")

    def imu_callback(self, msg):
        rospy.loginfo("IMU data received.")

    def set_initial_pose(self):
        # Create a PoseWithCovarianceStamped message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"  # Set to the correct frame

        # Example initial position and orientation
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        # Set the covariance
        pose_msg.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                     0, 0.1, 0, 0, 0, 0,
                                     0, 0, 0.1, 0, 0, 0,
                                     0, 0, 0, 0.1, 0, 0,
                                     0, 0, 0, 0, 0.1, 0,
                                     0, 0, 0, 0, 0, 0.1]

        # Publish the initial pose
        rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10).publish(pose_msg)
        rospy.loginfo("Initial pose published.")

if __name__ == '__main__':
    try:
        EKFLocalization()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

