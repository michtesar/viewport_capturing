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

rate = rospy.Rate(10)
while not rospy.is_shutdown():
	position = crc.group.get_current_pose("r1_ee")
	position.pose.position.y += 1.0
	rospy.loginfo("x: {0:.4f}, y: {1:.4f}, z:{2:.4f}, w:{2:.4f}".format(
		position.pose.orientation.x,
		position.pose.orientation.y,
		position.pose.orientation.z,
		position.pose.orientation.w
	))
	rate.sleep()
