#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Path

class DistanceAndTimeTracker:
    def __init__(self):
        # Initialize the node
        rospy.init_node('distance_and_time_tracker', anonymous=True)

        # Subscribers
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.status_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)  # Start tracking on new goal
        
        # Subscribe to both possible global planner topics
        self.navfn_plan_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.plan_callback)
        self.rrt_plan_sub = rospy.Subscriber('/move_base/RRTPlannerROS/global_plan', Path, self.plan_callback)

        # Variables to track distance and time
        self.start_time = None
        self.last_position = None
        self.total_distance = 0.0
        self.goal_reached = False
        self.goal_set = False  # New flag to indicate a goal has been set

        # Variables to track path planning time and distance
        self.plan_start_time = None
        self.plan_compute_time = None
        self.planned_path_distance = 0.0

    def goal_callback(self, msg):
        # Callback when a new goal is received
        rospy.loginfo("New goal received! Starting distance and time tracking.")
        self.reset_tracking()
        self.goal_set = True
        self.start_time = time.time()  # Record start time when the goal is set
        self.request_plan()  # Request a new plan

    def pose_callback(self, msg):
        # Get current position and calculate distance, only if goal is set
        if not self.goal_set:
            return

        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        if self.last_position is None:
            # Initialize the position on first callback after goal
            self.last_position = current_position
        else:
            # Calculate distance between current and last position
            distance = self.calculate_distance(self.last_position, current_position)
            self.total_distance += distance
            self.last_position = current_position

    def calculate_distance(self, pos1, pos2):
        # Euclidean distance between two points
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def plan_callback(self, msg):
        # Callback for when a path is planned
        if self.plan_start_time is not None:
            # Calculate the time taken to compute the path
            self.plan_compute_time = time.time() - self.plan_start_time
            rospy.loginfo("Planned path computation time: {:.2f} seconds".format(self.plan_compute_time))
            self.plan_start_time = None  # Reset after logging

            # Calculate the planned path distance
            self.planned_path_distance = self.calculate_path_distance(msg)
            rospy.loginfo("Planned path distance: {:.2f} meters".format(self.planned_path_distance))

    def calculate_path_distance(self, path):
        # Calculate the total length of the planned path
        total_distance = 0.0
        poses = path.poses
        
        # Sum up the distances between consecutive points in the path
        for i in range(1, len(poses)):
            pos1 = (poses[i-1].pose.position.x, poses[i-1].pose.position.y)
            pos2 = (poses[i].pose.position.x, poses[i].pose.position.y)
            total_distance += self.calculate_distance(pos1, pos2)
        
        return total_distance

    def status_callback(self, msg):
        # Check if the goal has been reached
        if msg.status.status == 3:  # Status 3 means goal reached
            self.goal_reached = True
            self.end_time = time.time()
            total_time = self.end_time - self.start_time
            
            # Log the final distance and time
            rospy.loginfo("Goal reached!")
            rospy.loginfo("Total distance traveled: {:.2f} meters".format(self.total_distance))
            rospy.loginfo("Total time taken: {:.2f} seconds".format(total_time))
            
            # Reset tracking variables after the goal is reached
            self.goal_set = False  # Reset the goal flag

    def request_plan(self):
        # Start tracking the plan time
        self.plan_start_time = time.time()
        rospy.loginfo("Requesting global planner to compute a path...")

    def reset_tracking(self):
        # Reset tracking variables for a new goal
        self.start_time = None
        self.last_position = None
        self.total_distance = 0.0
        self.goal_reached = False
        self.plan_start_time = None
        self.plan_compute_time = None
        self.planned_path_distance = 0.0

    def run(self):
        # Spin the node and wait for callbacks
        rospy.spin()

if __name__ == '__main__':
    try:
        tracker = DistanceAndTimeTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass

