#!/usr/bin/env python

"""
Implements the Path class used for creating robot trajectories in different frames.
"""

import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header, ColorRGBA


class Path:
    """
    Trajectory instance class
    """

    def __init__(self, l_id, a_id, scale, traj_name, orient_name, sub_name, frame_id, l_color, a_color):
        # Path line and arrow markers
        marker_line = Marker(type=4,
                             id=1,
                             lifetime=rospy.Duration(),
                             scale=scale,
                             header=Header(frame_id=frame_id),
                             color=l_color,
                             action=0)

        marker_arrow = Marker(type=0,
                              id=1,
                              lifetime=rospy.Duration(),
                              scale=Vector3(0.5, 0.1, 0.1),
                              header=Header(frame_id=frame_id),
                              color=a_color,
                              action=0)

        # Create necessary publishers and subscribers
        traj_pub = rospy.Publisher(traj_name, Marker, queue_size=0)
        orient_pub = rospy.Publisher(orient_name, Marker, queue_size=0)
        sub = rospy.Subscriber(sub_name, Odometry, self.callback, (marker_line, marker_arrow, traj_pub, orient_pub))
        rospy.spin()

    @staticmethod
    def callback(data, args):
        """
        Callback trajectory and orientation updater and publisher function.
        """

        trajectory_marker = args[0]
        orientation_marker = args[1]
        t_publisher = args[2]
        o_publisher = args[3]

        trajectory_marker.points.append(data.pose.pose.position)
        orientation_marker.pose = data.pose.pose

        t_publisher.publish(trajectory_marker)
        o_publisher.publish(orientation_marker)
