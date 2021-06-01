#!/usr/bin/env python

import path
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

# Initialize the true path node
rospy.init_node('true_path_node')

# Create the true path trajectory line
true_path = path.Path(3, 4, Vector3(0.05, 0, 0), '/true_path', '/true_orient', '/base_pose_ground_truth',
                      'odom', ColorRGBA(0.0, 1.0, 1.0, 0.8), ColorRGBA(0.0, 1.0, 1.0, 0.8))
