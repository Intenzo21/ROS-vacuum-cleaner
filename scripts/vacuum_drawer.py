#!/usr/local/bin/python

import rospy, cv2, yaml, sys, os
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

rospy.init_node("vacuum_drawer")

dirname = os.path.dirname(sys.argv[1])
with open(sys.argv[1], "r") as f:
    config = yaml.safe_load(f)

img = cv2.imread(os.path.join(dirname, config['image']))
origin = config['origin']
res = config['resolution']
vac_radius = int(rospy.get_param("vacuum_radius") / res)


# Note: for map server, origin is bottom left of image. For openCV it's the top left. So we need to reverse y axis
def translate_to_pixel(x, y):
    return int((x - origin[0]) / res), int(img.shape[0] - (y - origin[1]) / res)


vacuum = False


def handle_vacuum(b):
    global vacuum
    vacuum = b.data


def handle_bpgt(odom):
    global img
    if not vacuum:
        return
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    img = cv2.circle(img, translate_to_pixel(x, y), vac_radius, (255, 0, 0), -1)


pose_listener = rospy.Subscriber("/base_pose_ground_truth", Odometry, handle_bpgt)

vacuum_listener = rospy.Subscriber("/vacuum", Bool, handle_vacuum)

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    cv2.imshow("vacuuming", img)
    cv2.waitKey(1)
