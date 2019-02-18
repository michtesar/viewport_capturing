#!/usr/bin/python

import rospy
import numpy
from capek_pycommander.capek_robot import CapekRobotCommander
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
from copy import deepcopy
import math

# Initialize node and robot
rospy.init_node("collect_data")
crc = CapekRobotCommander("r1")
crc.group.set_planner_id("RRTConnectkConfigDefault")
crc.group.set_pose_reference_frame("r1_link_0")

center = Pose()
center.position.x = 0.5
center.position.y = 0.0
center.position.z = 0.0
center.orientation.y = 1.0

p1 = deepcopy(center)
p1.position.z = 0.3
p1.orientation.w = 0.0
p1.orientation.x = 0.1
p1.orientation.y = 1.0
p1.orientation.z = 0.0

crc.group.set_pose_target(p1)
crc.group.go(wait=True)
rospy.sleep(3)
