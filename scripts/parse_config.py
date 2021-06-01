#!/usr/bin/env python

"""
YAML file parser (config.yaml) to get all the ROS parameters needed.
Also, draws the goal points on the map.
"""

import rospy
import yaml
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from pathlib import Path

path = Path(__file__).parent / "../world/config.yaml"
with open(path, 'r') as stream:
    try:
        parsed_yaml_file = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# Assign the parsed yaml file data to variables
charge_speed = parsed_yaml_file["charge_speed"]
const_drain = parsed_yaml_file["const_drain"]
drive_drain = parsed_yaml_file["drive_drain"]
turn_drain = parsed_yaml_file["turn_drain"]
vacuum_drain = parsed_yaml_file["vacuum_drain"]
chargers = parsed_yaml_file["chargers"]
vacuum_radius = parsed_yaml_file["vacuum_radius"]
instructions = parsed_yaml_file["instructions"]

# Set rospy parameters (to roscore for later use)
rospy.set_param("charge_speed", charge_speed)
rospy.set_param("const_drain", const_drain)
rospy.set_param("drive_drain", drive_drain)
rospy.set_param("turn_drain", turn_drain)
rospy.set_param("vacuum_drain", vacuum_drain)
rospy.set_param("chargers", chargers)
rospy.set_param("vacuum_radius", vacuum_radius)
rospy.set_param("goals", instructions)


def wait_for_time():
    """
    Wait for simulated time to begin.                          
    """

    while rospy.Time().now().to_sec() == 0:
        pass


def show_goals_in_rviz(instructions, publisher):
    """
    Visualize goal markers in rviz
    """
    marker = Marker(type=6,
                    id=0,
                    lifetime=rospy.Duration(),
                    scale=Vector3(0.2, 0.2, 0.2),
                    header=Header(frame_id='odom'),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                    action=0)
    for i in range(len(instructions)):
        marker.points.append(Point(instructions[i][0], instructions[i][1], 0))
    while not rospy.is_shutdown():
        publisher.publish(marker)


def main():
    rospy.init_node('parse_config')
    wait_for_time()
    marker_publisher = rospy.Publisher('goal_markers', Marker, queue_size=5)
    rospy.sleep(0.5)
    show_goals_in_rviz(instructions, marker_publisher)


if __name__ == '__main__':
    main()
