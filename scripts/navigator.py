#!/usr/bin/env python

"""
Robot navigator module used to navigate the 
driving and vacuuming of the vacuum cleaner robot.
"""

import rospy
import tf  # Frame transformations

from math import pi, atan2
from help_funcs import eucledian_dist

import marker  # Creating markers
from sensor_msgs.msg import LaserScan  # Obstacle detection
from geometry_msgs.msg import Twist, Vector3  # Publishing velocities and creating markers
from nav_msgs.msg import Odometry  # Odometry subscriber
from std_msgs.msg import Bool  # Vacuuming


OBSTACLE_THRESHOLD = 0.147  # Minimum distance to obstacle


class Navigator:
	"""
	Class to navigate the robot (vacuum cleaner)
	"""

	def __init__(self):

		# Robot position variable
		self.rob_pose = rospy.get_param("/robot_start_pose", [2.000, 0.000, 0.000, 0.000])
		self.odom_pose = [0., 0., 0.]  # Odometry pose list
		self.theta = 0.  # Robot theta angle
		self.hz = rospy.Rate(10)

		self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

		# Publisher for the vacuum
		self.vac_pub=rospy.Publisher('/vacuum', Bool, queue_size=10)

		# Create the vacuum line marker which will be visualized only when vacuuming
		self.vacuum_line = marker.Markers(rgb=[.0, .0, 1.0], ns="v_line", frame="map", size=[.1, .1, .1])

		# Base odometry subscriber
		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.update_pose)

		# Laser scan subscriber
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
		self.vacuuming = False  # Boolean to enable vacuuming to take place

		# Twist message which expresses velocity in free space broken
		# into its linear and angular parts. (used in publishing robot
		# linear and angular velocities)
		self.vel_msg = Twist()

	def laser_callback(self, data):
		"""
		Process the laser scan data
		"""

		# Take the left laser scan ranges and if the minimum
		# scan range is equal or below the obstacle threshold
		# stop vacuuming (otherwise we will hit a wall)
		left_side = data.ranges[540:1080]
		if min(left_side) <= OBSTACLE_THRESHOLD:
			self.vacuuming = False

	def update_pose(self, odom_pose):
		"""
		Update the odometry pose (odometry sub callback function) 
		and visualize the vacuumed area in rviz
		"""

		pose = odom_pose.pose.pose
		pos	= pose.position
		ori	= pose.orientation
		trf = tf.transformations

		pos_z = trf.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]

		self.odom_pose = self.rob_pose = [pos.x, pos.y, pos_z]

		if self.vacuuming:
			self.vacuum_line.add_marker([self.rob_pose[0], self.rob_pose[1]])
		self.vacuum_line.draw_markers()

	def vc_freeze(self):
		"""
		Stop the vacuum cleaner robot
		"""

		self.vel_msg.angular.z = 0
		self.vel_msg.linear.x = 0
		self.vel_publisher.publish(self.vel_msg)

	def start_vacuuming(self, tsp_goal):
		"""
		Start spirally vacuuming the current goal room
		"""

		rospy.sleep(0.1)

		# Initialize vacuuming
		self.vacuuming = True
		self.vac_pub.publish(Bool(True))

		# Stop the robot inertia
		self.vc_freeze()
		# self.hz.sleep()

		while self.vacuuming:

			self.vel_msg.angular.z = -0.5  # Rotate clockwise at constant rate
			self.vel_msg.linear.x += 0.00075  # Increment forward velocity to achieve spiral movement
			self.vel_publisher.publish(self.vel_msg)  # Update velocity
			self.hz.sleep()  # Rate to control the loop (10 Hz)

		# Stop the vacuum drain and the robot
		self.vac_pub.publish(Bool(False))
		self.vc_freeze()

		r_x, r_y = self.rob_pose[0], self.rob_pose[1]  # Get the current robot (x, y) pose

		# Drive back to goal for better path calculation
		while eucledian_dist(self.rob_pose, tsp_goal) > 0.1:
			self.to_goal(tsp_goal)
			r_x, r_y = self.rob_pose[0], self.rob_pose[1]
			self.hz.sleep()

		self.vc_freeze() # Stop the robot inertia

	def to_goal(self, g):
		"""
		Move the robot to given goal location
		"""

		# Get the robot and goal (x,y) coordinates
		curr_x = self.rob_pose[0]
		curr_y = self.rob_pose[1]
		goal_x = g[0]
		goal_y = g[1]

		# Update the theta angle
		self.theta = atan2(goal_y - curr_y, goal_x - curr_x)
		yaw = self.rob_pose[2]  # Get the yaw of the robot

		# Calculate the difference angle difference to
		# know how much the robot should be rotated and adjust
		diff = self.theta - yaw
		if diff < -pi:
			diff += 2*pi
		if diff > pi:
			diff -= 2*pi

		self.vel_msg.angular.z = diff

		# Ensure smooth movement (adjust forward movement to turning ratio)
		steps = [round(x*.01, 2) for x in range(100)][::-1]

		for step in steps:
			if round(self.vel_msg.angular.z, 2) == step:
				self.vel_msg.linear.x = (1. - step)/10

		self.vel_publisher.publish(self.vel_msg)  # Publish the newly calculated robot velocity message

