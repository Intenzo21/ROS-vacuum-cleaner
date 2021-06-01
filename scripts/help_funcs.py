#!/usr/bin/env python

""" 
Helper function implementations
"""

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from math import sqrt, pi, fabs, atan2, sin, cos
import numpy as np


def eucledian_dist(pos, goal):
    """
    Return straight line distance between two points 
    """

    eucl_dist = sqrt((pos[0] - goal[0]) ** 2 + (pos[1] - goal[1]) ** 2)

    return eucl_dist


def trans_rot_to_pose(transl, rot):
    """ 
    Convert the received translation and rotation to a geometry_msgs/Pose message 
    """
    pose_msg = Pose(position=Point(x=transl[0], y=transl[1], z=transl[2]),
                    orientation=Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3]))

    return pose_msg


def invert_pose_tf(pose):
    """ 
    Invert a pose transform
    """

    transl = np.zeros((4, 1))
    transl[0] = -pose.position.x
    transl[1] = -pose.position.y
    transl[2] = -pose.position.z
    transl[3] = 1.0

    rot = (pose.orientation.x,
           pose.orientation.y,
           pose.orientation.z,
           pose.orientation.w)

    euler_angle = euler_from_quaternion(rot)
    rot = np.transpose(rotation_matrix(euler_angle[2], [0, 0, 1]))  # the angle is a yaw
    trfd_transl = rot.dot(transl)

    transl = (trfd_transl[0], trfd_transl[1], trfd_transl[2])
    rot = quaternion_from_matrix(rot)
    return transl, rot


def pose_to_triple(pose):
    """ 
    Convert pose (geometry_msgs.Pose) to a (x,y,yaw) triple
    """

    orient_tpl = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orient_tpl)
    triple = (pose.position.x, pose.position.y, angles[2])
    return triple


def norm_angle(angl):
    """ 
    Maps the angle 'angl' to the [-pi, pi] range
    """

    return atan2(sin(angl), cos(angl))


def get_rot_mat(theta):
    """
    Return a 2x2 rotation matrix
    """

    angl_sin = np.sin(theta)
    angl_cos = np.cos(theta)

    return np.array([[angl_cos, -angl_sin],
                     [angl_sin, angl_cos]])


def angle_diff(a, b):
    """ 
    Calculates the difference between angles a and b 
    based on the closest rotation from angle a to b.
    """

    a = norm_angle(a)
    b = norm_angle(b)
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2
