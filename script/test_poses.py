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
rospy.init_node("test_poses")
crc = CapekRobotCommander("r1")
crc.group.set_planner_id("RRTConnectkConfigDefault")
crc.group.set_pose_reference_frame("r1_link_0")
crc.set_speed(0.4)

pose = Pose()

pos_x, pos_y, pos_z = 0, 0, 0
qua_x, qua_y, qua_z, qua_w = 0, 0, 0, 0

rate = rospy.Rate(10)
while not rospy.is_shutdown():
	choice = input("(1) set pose, (2) set position, (3) set orientation, (4) move to L, (5) euler\n")

	if choice == 1:
		rospy.loginfo("Plan by Pose from base link frame")
		pos_x = input("Position x: ")
		pos_y = input("Position y: ")
		pos_z = input("Position z: ")
		qua_w = input("Quaternion w: ")
		qua_x = input("Quaternion x: ")
		qua_y = input("Quaternion y: ")
		qua_z = input("Quaternion z: ")
	elif choice == 2:	
		pos_x = input("Position x: ")
		pos_y = input("Position y: ")
		pos_z = input("Position z: ")
	elif choice == 3:
		qua_w = input("Quaternion w: ")
		qua_x = input("Quaternion x: ")
		qua_y = input("Quaternion y: ")
		qua_z = input("Quaternion z: ")
	elif choice == 4:
		crc.move_l_position()
	elif choice == 5:
		pos_x = input("Position x: ")
		pos_y = input("Position y: ")
		pos_z = input("Position z: ")
		x = input("Euler x: ")
		y = input("Euler y: ")
		z = input("Euler z: ")
		euler = [math.radians(x), math.radians(y), math.radians(z)]
		from tf.transformations import quaternion_from_euler
		quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
		qua_x = quaternion[0]
		qua_y = quaternion[1]
		qua_z = quaternion[2]
		qua_w = quaternion[3]
	else:
		rospy.logwarn("Enter only valid options")

	if not choice == 4:
		this_pose = deepcopy(pose)
		this_pose.position.x = float(pos_x)
		this_pose.position.y = float(pos_y)
		this_pose.position.z = float(pos_z)
		this_pose.orientation.w = float(qua_w)
		this_pose.orientation.x = float(qua_x)
		this_pose.orientation.y = float(qua_y)
		this_pose.orientation.z = float(qua_z)

		rospy.loginfo(this_pose)

		crc.group.set_pose_target(this_pose)
		crc.group.go(wait=True)	
