#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from message_filters import Subscriber, ApproximateTimeSynchronizer
import time
import threading
import matplotlib

# Set matplotlib backend to ensure the plot appears
matplotlib.use('TkAgg')

class LocalizationErrorPlotter:
    def __init__(self):
        rospy.init_node('localization_error_plotter', anonymous=True)

        # Lists to store error, uncertainty, and time data
        self.errors = []
        self.uncertainties = []
        self.timestamps = []

        # Subscribers with message filters for synchronization
        self.amcl_sub = Subscriber('/amcl_pose', PoseWithCovarianceStamped)
        self.gt_sub = Subscriber('/ground_truth_path', Path)

        # Synchronize AMCL and ground truth messages
        self.ts = ApproximateTimeSynchronizer([self.amcl_sub, self.gt_sub], queue_size=50, slop=1.0)
        self.ts.registerCallback(self.sync_callback)

        self.start_time = time.time()

        # Initialize live plot
        self.fig, self.ax = plt.subplots()
        self.error_line, = self.ax.plot([], [], label="Position Error (m)", color="red")
        self.uncertainty_line, = self.ax.plot([], [], label="Uncertainty (m)", color="blue")

        self.ax.set_xlim(0, 10)  # Will adjust dynamically as data comes in
        self.ax.set_ylim(0, 1)   # Adjust y limits based on data
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Error / Uncertainty (m)")
        self.ax.legend()
        plt.title("Live Localization Error and Uncertainty")

        # Set up the animation
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=500)  # Update every second

        rospy.loginfo("Localization Error Plotter Initialized")

    def sync_callback(self, amcl_msg, gt_msg):
        rospy.loginfo("Receiving synchronized data.")  # Debugging: check if data is being received

        # Store the latest AMCL pose
        self.amcl_pose = amcl_msg.pose.pose

        # Calculate uncertainty from the covariance matrix
        covariance = np.array(amcl_msg.pose.covariance).reshape(6, 6)
        uncertainty = np.sqrt(covariance[0, 0] + covariance[1, 1])  # x, y uncertainty
        self.uncertainties.append(uncertainty)

        # Get the last pose from the ground truth path
        self.ground_truth_pose = gt_msg.poses[-1].pose  # Take the last pose in the Path

        # Compute error
        self.compute_error()

    def compute_error(self):
        # Compute the error between AMCL pose and ground truth pose
        error_x = self.amcl_pose.position.x - self.ground_truth_pose.position.x
        error_y = self.amcl_pose.position.y - self.ground_truth_pose.position.y
        position_error = np.sqrt(error_x**2 + error_y**2)
        self.errors.append(position_error)

        # Append the current timestamp
        current_time = time.time() - self.start_time
        self.timestamps.append(current_time)

    def update_plot(self, frame):
        if self.timestamps:  # Only update if we have data
            # Set new data for both lines
            self.error_line.set_data(self.timestamps, self.errors)
            self.uncertainty_line.set_data(self.timestamps, self.uncertainties)

            # Dynamically adjust the x and y axis limits
            self.ax.set_xlim(0, max(self.timestamps) + 1)
            self.ax.set_ylim(0, max(max(self.errors, default=0), max(self.uncertainties, default=0)) + 0.1)

        return self.error_line, self.uncertainty_line

    def run(self):
        # Create a separate thread for ROS so it doesn't block the plot
        ros_thread = threading.Thread(target=rospy.spin)
        ros_thread.start()

        # Start live plotting
        plt.show()

        # Wait for the ROS thread to complete (on shutdown)
        ros_thread.join()

if __name__ == '__main__':
    plotter = LocalizationErrorPlotter()

    # Start the plotter
    plotter.run()

