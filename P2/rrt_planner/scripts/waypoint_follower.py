#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler

class WaypointFollower:
    def __init__(self):
        # Initialize the node
        rospy.init_node('waypoint_follower')

        # Create action client to communicate with move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Create a publisher for the markers
        self.marker_pub = rospy.Publisher('/waypoint_markers', Marker, queue_size=10)

        # Ensure the publisher is ready before publishing
        rospy.sleep(1)  # Wait for the publisher to be fully initialized

        # Define the waypoints (x, y, and yaw in radians)
        self.waypoints = [
            {'x': 0.5, 'y': -5.0, 'yaw': 0.0},
            {'x': 1.5, 'y': -5.5, 'yaw': 0.0},
            {'x': 3.0, 'y': -4.5, 'yaw': 3.14},
            {'x': 3.3, 'y': -2.0, 'yaw': 0.0}
        ]
        # Publish all markers before starting the navigation
        self.publish_all_markers()

        # Execute the waypoint navigation
        self.execute_waypoints()

    def execute_waypoints(self):
        # Loop through each waypoint and move the robot to it
        for i, waypoint in enumerate(self.waypoints):
            rospy.loginfo(f"Moving to waypoint {i+1}: {waypoint}")
            if self.move_to_goal(waypoint['x'], waypoint['y'], waypoint['yaw']):
                rospy.loginfo(f"Reached waypoint {i+1}")
            else:
                rospy.logerr(f"Failed to reach waypoint {i+1}")
                break  # Stop if the robot fails to reach the waypoint

    def move_to_goal(self, x, y, yaw):
        # Create a new goal to send to move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Set the frame of the goal
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the position of the goal
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        # Set the orientation of the goal (converted from yaw)
        quat = self.yaw_to_quaternion(yaw)
        goal.target_pose.pose.orientation = quat

        # Send the goal to move_base and wait for the result
        self.client.send_goal(goal)
        self.client.wait_for_result()

        # Check if the goal was successfully reached
        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def yaw_to_quaternion(self, yaw):
        # Convert yaw (in radians) to a quaternion for goal orientation
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        return Pose().orientation.__class__(*quat)

    def publish_marker(self, x, y, index):
        # Publish a marker at the given position (x, y)
        rospy.loginfo(f"Publishing marker at ({x}, {y}) with ID {index}")
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoints"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the pose of the marker
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0  # No rotation for the sphere

        # Set marker scale (increase size for better visibility)
        marker.scale.x = 0.1  # Larger size for better visibility
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set marker color (R, G, B, Alpha)
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Set the lifetime of the marker (0 means it will persist)
        marker.lifetime = rospy.Duration(0)

        # Publish the marker
        self.marker_pub.publish(marker)

    def publish_all_markers(self):
        # Publish markers for all waypoints at the beginning
        for i, waypoint in enumerate(self.waypoints):
            self.publish_marker(waypoint['x'], waypoint['y'], i)
            rospy.loginfo(f"Published marker for waypoint {i+1} at position ({waypoint['x']}, {waypoint['y']})")

if __name__ == '__main__':
    try:
        WaypointFollower()
    except rospy.ROSInterruptException:
        pass

