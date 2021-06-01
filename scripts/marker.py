#!/usr/bin/env python

""" 
Marker class used for creating the goal path and the vacuumed area line
"""

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class Markers:
    def __init__(self, rgb, ns, frame, size, m_type=Marker.SPHERE):
        self.scale = size
        self.id = 0
        self.namespace = ns
        self.frame = frame
        self.type = m_type
        self.rgb = rgb
        self.markers = []
        self.publisher = rospy.Publisher(self.namespace, MarkerArray, queue_size=10)

    def add_marker(self, pos, ortn=None):
        """
		Create a marker instance and add it to the markers list
		"""

        m = Marker()
        m.header.frame_id = self.frame
        m.ns = self.namespace
        m.id = self.id
        m.type = self.type
        m.action = m.ADD
        m.scale.x = self.scale[0]
        m.scale.y = self.scale[1]
        m.scale.z = self.scale[2]
        m.color.r = self.rgb[0]
        m.color.g = self.rgb[1]
        m.color.b = self.rgb[2]
        m.color.a = 1.

        if ortn != None:
            m.pose.position = pos
            m.pose.orientation = ortn
        else:
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]

            if len(pos) == 3:
                m.pose.position.z = pos[2]

        self.markers.append(m)
        self.id += 1

    def add_line(self, points):
        """
		Create a marker instance and connect all marker points
		"""

        m = Marker()
        m.header.frame_id = self.frame
        m.ns = self.namespace
        m.id = self.id
        m.type = m.LINE_STRIP
        m.action = m.ADD
        m.scale.x = self.scale[0]
        m.scale.y = self.scale[1]
        m.scale.z = self.scale[2]
        m.color.r = self.rgb[0]
        m.color.g = self.rgb[1]
        m.color.b = self.rgb[2]
        m.color.a = 1.

        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            m.points.append(p)

        self.markers.append(m)
        self.id += 1

    def draw_markers(self):
        """
		Create a marker array and publish it
		"""

        mark_array = MarkerArray()

        for marker in self.markers:
            mark_array.markers.append(marker)

        self.publisher.publish(mark_array)

    def clear_markers(self):
        """
		Clear all the marker array markers
		"""

        mark_array = MarkerArray()

        if self.markers:
            for marker in self.markers:
                marker.action = marker.DELETE
                mark_array.markers.append(marker)

        self.publisher.publish(mark_array)
        self.id = 0
        self.markers = []
