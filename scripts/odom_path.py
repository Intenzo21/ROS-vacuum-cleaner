#!/usr/bin/env python

import path
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

# Initialize the odometry path node
rospy.init_node('odom_path_node')

# Create the odometry path trajectory line
odom_path = path.Path(1, 2, Vector3(0.05, 0, 0), '/believed_path', '/believed_orient', '/odom',
                      'odom', ColorRGBA(1.0, 1.0, 0.0, 0.8), ColorRGBA(1.0, 1.0, 1.0, 0.8))
