#! /usr/bin/env python3
'''
scan photoacoustics
'''
import copy
import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from rv_msgs.msg import MoveToJointPoseAction, MoveToJointPoseGoal
from tf import transformations


class PAScan():  # base class for OCT & PA?
  O_T_EE_entry = np.array([[0, -1, 0, 0.3], [-1, 0, 0, 0.0], [0, 0, -1, 0.4], [0, 0, 0, 1]])
  q_home = [0.0, -np.pi/6, 0.0, -2*np.pi/3, 0.0, np.pi/2, -np.pi/3]  # home configureation

  def __init__(self):
    self.O_T_EE = None
    self.scan_vel = Twist()
    self.scan_vel_last = Twist()
    self.vel_pub = rospy.Publisher('arm/cartesian/velocity', TwistStamped, queue_size=1)
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.franka_pose_cb)
    self.cart_pose_cli = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
    self.joint_pos_cli = actionlib.SimpleActionClient('/arm/joint/pose', MoveToJointPoseAction)
    self.cart_pose_cli.wait_for_server()
    self.joint_pos_cli.wait_for_server()
    self.rate = rospy.Rate(500)
    while self.O_T_EE is None is None:
      if rospy.is_shutdown():
        return
    print("robot state received \nconnection establised")

  def go_to_home_config(self):
    goal = MoveToJointPoseGoal(joints=self.q_home)
    self.joint_pos_cli.send_goal(goal)
    self.joint_pos_cli.wait_for_result()
    with np.printoptions(precision=4, suppress=True):
      print('reached home configuration @:\n', self.O_T_EE)

  def go_to_entry_pose(self):
    entry_pose = PoseStamped()
    entry_pose.header.frame_id = 'panda_link0'
    entry_rot = transformations.quaternion_from_matrix(self.O_T_EE_entry)
    entry_pose.pose.orientation.x = entry_rot[0]
    entry_pose.pose.orientation.y = entry_rot[1]
    entry_pose.pose.orientation.z = entry_rot[2]
    entry_pose.pose.orientation.w = entry_rot[3]
    entry_pose.pose.position.x = self.O_T_EE_entry[0, -1]
    entry_pose.pose.position.y = self.O_T_EE_entry[1, -1]
    entry_pose.pose.position.z = self.O_T_EE_entry[2, -1]
    goal = MoveToPoseGoal(goal_pose=entry_pose)
    self.cart_pose_cli.send_goal(goal)
    self.cart_pose_cli.wait_for_result()
    with np.printoptions(precision=4, suppress=True):
      print('reached entry pose @:\n', self.O_T_EE)

  def do_scan_process(self):  # blocking behavior
    x_end = 0.4
    self.scan_vel.linear.x = 0.01
    while not rospy.is_shutdown():
      scan_vel_filtered = self.IIR_filter()
      vel_msg = TwistStamped(twist=scan_vel_filtered)
      self.vel_pub.publish(vel_msg)
      self.scan_vel_last = copy.deepcopy(scan_vel_filtered)
      if self.O_T_EE[0, -1] >= x_end:
        print('scan complete')
        break
      self.rate.sleep()

  def IIR_filter(self) -> Twist:
    """
    apply IIR filter on velocity commands
    """
    p = 0.125
    filtered = Twist()
    filtered.linear.x = p*self.scan_vel.linear.x + (1-p)*self.scan_vel_last.linear.x
    filtered.linear.y = p*self.scan_vel.linear.y + (1-p)*self.scan_vel_last.linear.y
    filtered.linear.z = p*self.scan_vel.linear.z + (1-p)*self.scan_vel_last.linear.z
    filtered.angular.x = p*self.scan_vel.angular.x + (1-p)*self.scan_vel_last.angular.x
    filtered.angular.y = p*self.scan_vel.angular.y + (1-p)*self.scan_vel_last.angular.y
    filtered.angular.z = p*self.scan_vel.angular.z + (1-p)*self.scan_vel_last.angular.z
    return filtered

  def franka_pose_cb(self, msg: FrankaState):
    curr_ee_pos = msg.O_T_EE  # inv 4x4 matrix
    self.O_T_EE = np.array([curr_ee_pos[0:4], curr_ee_pos[4:8],
                            curr_ee_pos[8:12], curr_ee_pos[12:16]]).transpose()


if __name__ == "__main__":
  rospy.init_node('franka_pa_scan')
  scan = PAScan()
  scan.go_to_home_config()
  scan.go_to_entry_pose()
  scan.do_scan_process()
