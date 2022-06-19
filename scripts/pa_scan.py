#! /usr/bin/env python3
'''
scan photoacoustics
'''
import copy
import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped
from tf import transformations
import os
import json

useArmer = False
if useArmer:
  from armer_msgs.msg import MoveToPoseAction, MoveToPoseGoal
else:
  from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal

# read transformation from ee to 8th link
ee_config_file = '../config/pa_config.json'
with open(os.path.join(os.path.dirname(__file__), ee_config_file)) as f:
  ee_config = json.load(f)['transformation']
T_8_EE = np.array([ee_config[0:4], ee_config[4:8], ee_config[8:12], ee_config[12:16]]).transpose()

T_O_EE_d = np.array([[0, -1, 0, 0.4],
                    [-1, 0, 0, 0.0],
                    [0, 0, -1, 0.3],
                    [0, 0, 0, 1]])

if useArmer:
  T_EE_8 = np.linalg.inv(T_8_EE)
  T_O_8_d = np.matmul(T_O_EE_d, T_EE_8)
  print('T_O_EE:\n', np.matmul(T_O_8_d, T_8_EE))
  print('T_O_8:\n', T_O_8_d)
  quat = transformations.quaternion_from_matrix(T_O_8_d)
else:
  quat = transformations.quaternion_from_matrix(T_O_EE_d)
print('quat:\n', quat)
target = PoseStamped()
target.header.frame_id = 'panda_link0'
target.pose.orientation.x = quat[0]
target.pose.orientation.y = quat[1]
target.pose.orientation.z = quat[2]
target.pose.orientation.w = quat[3]
if useArmer:
  target.pose.position.x = T_O_8_d[0, -1]
  target.pose.position.y = T_O_8_d[1, -1]
  target.pose.position.z = T_O_8_d[2, -1]
else:
  target.pose.position.x = T_O_EE_d[0, -1]
  target.pose.position.y = T_O_EE_d[1, -1]
  target.pose.position.z = T_O_EE_d[2, -1]
print('pose goal:\n', target)

rospy.init_node('franka_pa_scan')
client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
client.wait_for_server()

if useArmer:
  goal = MoveToPoseGoal(pose_stamped=target)
else:
  goal = MoveToPoseGoal(goal_pose=target)
client.send_goal(goal)
client.wait_for_result()
