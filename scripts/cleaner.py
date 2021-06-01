#!/usr/bin/env python

"""
Vacuum cleaner implementation. This module is employed in driving the vacuum cleaner to the
specified goals while 'spirally' vacuuming the rooms where the goal are located
"""

import rospy
from math import inf

import navigator
import marker
import pathfinder
import path

from help_funcs import eucledian_dist
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA, Bool, Float32
from queue import PriorityQueue


class VCleaner:
	"""
	Vacuum cleaner class
	"""

	def __init__(self):

		# Initialize the vacuum cleaner node
		rospy.init_node("cleaner")

		# Set the initial pose of the vacuum cleaner
		self.curr_pose = rospy.get_param("robot_start_pose", [2.000, 0.000, 0.000, 0.000])

		# Get the map cells
		rospy.wait_for_service('/static_map')
		self.map_cells = rospy.ServiceProxy('/static_map', GetMap)().map

		# Initialize the vacuum cleaner navigator
		self.navigator = navigator.Navigator()

		# Return the list of goal and charger location coordinates.
		# These parameters are set in the 'parse_config.py' to the roscore.
		self.goal_coords = rospy.get_param("/goals")
		self.charger_locs = rospy.get_param("/chargers")

		self.hz = rospy.Rate(10)  # Assign frequency for later use in while loops
		
		# Publisher for the vacuum 
		self.vac_pub=rospy.Publisher('/vacuum', Bool, queue_size=10)

		rospy.Subscriber("/base_pose_ground_truth", Odometry, self.update_pose)
		# rospy.Subscriber("/battery", Float32, self.needs_charging)

		# TODO: Complete driving to chargers
		self.at_charger = False
		self.closest_charger = None

		# Green path to goal (marker array)
		self.goal_path = marker.Markers(rgb=[0., 1., 0.], ns="path", frame="odom", size=[.07, .07, .1])

		# Room floor highlight marker
		self.room_floor = marker.Markers(rgb=[0.5, 1., 0.5], ns="room", frame="odom", size=[2, 2, .0], m_type=1)

		# Prioritize goals
		self.goals_queue = PriorityQueue()
		self.prioritize_goals()
				
		# Find the best route by applying TSP
		self.tsp_route = pathfinder.adapted_tsp((self.curr_pose[0], self.curr_pose[1]), self.goals_queue)

	def prioritize_goals(self):
		"""
		Method to prioritize goals (put the closest goal first)
		"""

		for goal in self.goal_coords:
			self.goals_queue.put((0,(goal[0],goal[1])))

	def update_pose(self, odom):
		"""
		Update the odometry pose (callback method)
		"""

		pos = odom.pose.pose.position
		self.curr_pose = (pos.x, pos.y)

	def needs_charging(self, battery):
		"""
		Check if the vacuum cleaner battery needs charging.
		If so get the closest charging point and drive to there.
		TODO: implement A*
		"""
		if battery.data <= 50 and not self.at_charger:

			# Distance to closest charger
			ch_loc, closest_ch = None, inf
			for ch in self.charger_locs:
				closest_dist = eucledian_dist(self.curr_pose, ch)
				if closest_dist <= closest_ch:
					closest_ch = closest_dist
					ch_loc = ch

			self.navigator.vac_pub.publish(False)
			self.navigator.vacuuming = False

			while eucledian_dist(self.curr_pose, ch_loc) > 0.081:
				self.navigator.to_goal(ch_loc)

			self.at_charger = True

		if self.at_charger:
			if battery.data < 99:
				print(battery.data)
				self.navigator.vc_freeze()
			else:
				self.at_charger = False

	def run_vc(self):
		"""
		Run the vacuum cleaner (go from goal to goal starting from the closest)
		"""

		for tsp_goal in self.tsp_route:

			print("Going to goal:", tsp_goal)
			self.room_floor.add_marker((tsp_goal[0] + 0.1, tsp_goal[1]))
			
			# Round the start location coordinates
			start_loc = tuple([round(x,1) for x in self.curr_pose])
			
			# Round the goal coordinates for the A* search
			goal_rounded = tuple([round(x, 1) for x in tsp_goal])

			curr_path = pathfinder.a_star(start_loc, goal_rounded, self.map_cells)
			
			# Add the goal location
			if tsp_goal not in curr_path:
				curr_path.append(tsp_goal)
			
			# Connect the goal path points
			self.goal_path.add_line(curr_path)
			
			# For every point in the current goal path
			for goal in curr_path:
					
				while not rospy.is_shutdown():
					
					# If a point from the path is reached then go to next one
					# Use Eucledian distance to goal point from current point
					if eucledian_dist(self.curr_pose, goal) < .081:

						# If the current goal is the current path end goal (drawn in rviz)
						if goal == tsp_goal:
							self.room_floor.clear_markers()  # Remove the room highlight marker
							self.navigator.start_vacuuming(tsp_goal)  # Start vacuuming
							
						break  # Skip to next goal
					
					self.room_floor.draw_markers()
					self.goal_path.draw_markers()
					self.hz.sleep()
					
					# if not self.at_charger:
					self.navigator.to_goal(goal)  # Navigate the robot to the goal point
					self.hz.sleep()	


if __name__ == '__main__':
	# Initialize the vacuum cleaner object which runs the program
	vcleaner = VCleaner()
	vcleaner.run_vc()  # Run the vacuum cleaner

