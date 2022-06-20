#! /usr/bin/env python3
'''
publish franka state using ros built in message types for MATLAB
'''
import time
import rospy
import numpy as np
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64MultiArray


class pub_franka_state():
  T_O_ee = None
  franka_state_msg = Float64MultiArray()
  update_period = 3   # [sec]

  def __init__(self, pub_rate=1000):
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.ee_callback)
    self.franka_state_pub = rospy.Publisher('franka_state_custom', Float64MultiArray, queue_size=1)
    self.rate = rospy.Rate(pub_rate)
    print("publishing franka state ...")
    while not rospy.is_shutdown():
      # now = time.time()
      if self.T_O_ee is not None:
        self.franka_state_msg.data = self.T_O_ee.flatten()
        self.franka_state_pub.publish(self.franka_state_msg)
      else:
        pass
      self.rate.sleep()
      # print('elapsed:', 1/(time.time()-now))

  def disp_robot_state(self):
    """
    print robot state in terminal
    """
    print('T_O_EE\n', self.T_O_ee)

  def ee_callback(self, msg):
    # EE_pos = msg.O_T_EE_d  # inv 4x4 matrix
    EE_pos = msg.O_T_EE  # inv 4x4 matrix
    self.T_O_ee = np.array([EE_pos[0:4], EE_pos[4:8], EE_pos[8:12], EE_pos[12:16]]).transpose()


if __name__ == "__main__":
  rospy.init_node('publish_franka_state', anonymous=True)
  pub_franka_state()
