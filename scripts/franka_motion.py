#! /usr/bin/env python3
import copy
import math
import time
import rospy
import actionlib
import numpy as np
from std_msgs.msg import Bool
from franka_msgs.msg import FrankaState


if __name__ == "__main__":
  rospy.init_node('franka_motion', anonymous=True)
