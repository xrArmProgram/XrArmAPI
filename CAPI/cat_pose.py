#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import cv2
import numpy as np
import cv2 as cv
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped, Pose 
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
import time
import tf2_ros
import tf2_geometry_msgs
from move_xr2_ik import MoveMK2ik
from control_msgs.msg import GripperCommand
from std_msgs.msg import UInt8
import signal


rospy.init_node("pos_arm")
group=moveit_commander.MoveGroupCommander('arm')
print(group.get_current_pose().pose)
