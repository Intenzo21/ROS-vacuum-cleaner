#!/usr/local/bin/python
import rospy
import math
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
import tf


### Publishes to the /battery topic which returns the percentage of battery remaining.

def distance(x1, x2, y1, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class Charger:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

    def charging(self, x, y):
        return distance(self.x, x, self.y, y) < self.r


rospy.init_node("battery")

CHARGE_SPEED = rospy.get_param("charge_speed", 1)
CONST_DRAIN = rospy.get_param("const_drain", 0.001)
DRIVE_DRAIN = rospy.get_param("drive_drain", 0.1)
TURN_DRAIN = rospy.get_param("turn_drain", 0.05)
VACUUM_DRAIN = rospy.get_param("vacuum_drain", 0.5)

chargers = set()
for c in rospy.get_param("chargers"):
    chargers.add(Charger(c[0], c[1], c[2]))

battery = 100
vacuuming = False  # is the vacuum on?

old_pose = None
old_time = None


def handle_new_pose(pose, time):
    global old_pose, old_time, battery
    if old_pose is None:
        old_pose = pose
        old_time = time
        return
    dt = time - old_time
    dx = pose.position.x - old_pose.position.x
    dy = pose.position.y - old_pose.position.y
    dz = pose.position.z - old_pose.position.z
    orient_old = old_pose.orientation
    orient = pose.orientation
    q1 = [orient_old.x, orient_old.y, orient_old.z, orient_old.w]
    q2 = [orient.x, orient.y, orient.z, -orient.w]
    turn = tf.transformations.quaternion_multiply(q1, q2)
    (p, r, y) = tf.transformations.euler_from_quaternion(turn)
    lin = math.sqrt(dx * dx + dy * dy + dz * dz)
    ang = abs(p) + abs(r) + abs(y)
    battery -= DRIVE_DRAIN * lin + ang * TURN_DRAIN
    for c in chargers:
        if c.charging(pose.position.x, pose.position.y):
            t = time - old_time
            t = t.to_sec() + (t.to_nsec() / 1000000000.0)
            battery += t * CHARGE_SPEED
    if battery > 100.0:
        battery = 100.0
    old_pose = pose
    old_time = time


def handle_command_vel(twist):
    if battery > 0:
        cmd_vel_pub.publish(twist)


def handle_vacuum(vac):
    global vacuuming
    vacuuming = vac.data


def handle_bpgt(bpgt):
    pose = bpgt.pose.pose
    time = bpgt.header.stamp
    handle_new_pose(pose, time)


cmd_vel_pub = rospy.Publisher('/cmd_vel_out', Twist, queue_size=10)
bat_pub = rospy.Publisher('/battery', Float32, queue_size=10)
cmd_vel_sub = rospy.Subscriber('/cmd_vel_in', Twist,
                               handle_command_vel)  # listen to inputs to prevent the robot from moving if no battery
pose_listener = rospy.Subscriber("/base_pose_ground_truth", Odometry,
                                 handle_bpgt)  # listen to what the robot pose is to determine if we're at a charger
# and how far we've moved
vacuum_listener = rospy.Subscriber("/vacuum", Bool, handle_vacuum)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    battery -= CONST_DRAIN
    if vacuuming:
        battery -= VACUUM_DRAIN
    if battery < 0.0:
        battery = 0
    bat_pub.publish(battery)
    r.sleep()
