#!/usr/bin/env python
import time
import sys
import rospy
import numpy as np
from moveit_python import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from tf.msg import tfMessage

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
				"wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

INITIAL_POSITION = [0, -np.pi * 0.5, 0, 0, 0, 0]

# https://github.com/ros-planning/moveit_commander/blob/0654bef18b08c43d34111764905488de16f3fabb/src/moveit_commander/planning_scene_interface.py

class UpdateScene():

	def __init__(self):
		rospy.init_node("constraint")

		scene = PlanningSceneInterface("base_link")
		scene.removeCollisionObject("ground")

		scene.addCube("ground", 2, 0, 0, -1)

	rospy.spin()

if __name__ == "__main__":
	UpdateScene()