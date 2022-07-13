#! /usr/bin/env python3
'''
scan photoacoustics
'''
import os
import csv
import time
import copy
import rospy
from datetime import date
import numpy as np
import actionlib
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from franka_msgs.msg import FrankaState
from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from rv_msgs.msg import MoveToJointPoseAction, MoveToJointPoseGoal
from tf import transformations


class PAScan():  # base class for OCT & PA?
  __init_x = 0.5
  __init_z = 0.19  # 0.14
  __init_y = 0.0
  O_T_EE_entry = np.array([[0, -1, 0, __init_x], [-1, 0, 0, __init_y], [0, 0, -1, __init_z], [0, 0, 0, 1]])
  q_home = [0.0, -np.pi/6, 0.0, -2*np.pi/3, 0.0, np.pi/2, -np.pi/3]  # home configureation
  robot_logger_path = os.path.join(os.path.dirname(__file__), '../data/franka_state/')
  robot_log_file = '{}-franka_state.csv'.format(date.today())

  def __init__(self):
    if not os.path.exists(self.robot_logger_path):
      os.makedirs(self.robot_logger_path)
    self.O_T_EE = None
    self.scan_vel = Twist()
    self.scan_vel_last = Twist()

    # ===== for communication with Verasonics
    self.franka_status = False
    self.pa_status = False
    self.franka_status_pub = rospy.Publisher('Franka_ready_flag', Bool, queue_size=1)
    rospy.Subscriber('PA_ready_flag', Bool, self.pa_status_cb)

    self.vel_pub = rospy.Publisher('arm/cartesian/velocity', TwistStamped, queue_size=1)
    rospy.Subscriber('franka_state_controller/franka_states', FrankaState, self.franka_pose_cb)
    self.cart_pose_cli = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
    self.joint_pos_cli = actionlib.SimpleActionClient('/arm/joint/pose', MoveToJointPoseAction)
    self.cart_pose_cli.wait_for_server()
    self.joint_pos_cli.wait_for_server()
    self.rate = rospy.Rate(1000)
    while self.O_T_EE is None is None:
      if rospy.is_shutdown():
        return
    print("robot state received \nconnection establised")

  def get_entry_pose(self) -> np.array:
    return self.O_T_EE_entry

  def set_entry_pose(self, O_T_EE_entry: np.array):
    assert(abs(np.linalg.det(O_T_EE_entry) - 1) <= 1e-4)
    self.O_T_EE_entry = copy.deepcopy(O_T_EE_entry)

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

  def do_scan_process_blind(self, scan_path_length=0.05, scan_vel=1e-3):  # blocking behavior
    x_end = self.O_T_EE_entry[0, -1] + scan_path_length
    self.scan_vel.linear.x = scan_vel
    while not rospy.is_shutdown():
      # print('curr x: ', self.O_T_EE[0, -1])
      scan_vel_filtered = self.IIR_filter()
      vel_msg = TwistStamped(twist=scan_vel_filtered)
      self.vel_pub.publish(vel_msg)
      self.scan_vel_last = copy.deepcopy(scan_vel_filtered)
      if self.O_T_EE[0, -1] >= x_end:
        print('complete one round')
        break
      self.rate.sleep()

  def do_scan_process(self, scan_path_length=0.08, scan_path_incre=0.0005, scan_vel=2.5e-4):
    x_end = self.O_T_EE[0, -1] + scan_path_length
    x_next = self.O_T_EE[0, -1] + scan_path_incre
    self.scan_vel.linear.x = scan_vel
    self.franka_status = False
    pa_timeout = 500
    stop_count = 0
    # blocking behavior
    while not rospy.is_shutdown():
      if self.O_T_EE[0, -1] < x_next:
        print('x: {:.6f}, x next: {:.6f}, x end: {:.6f}, '.format
              (self.O_T_EE[0, -1], x_next, x_end), 'stop count: ', stop_count)
        self.franka_status = False
        vel_msg = TwistStamped(twist=self.scan_vel)
      else:
        self.franka_status = True
        for i in range(pa_timeout):
          self.franka_status_pub.publish(Bool(self.franka_status))
        if self.pa_status:
          stop_count += 1
          x_next = self.O_T_EE[0, -1] + scan_path_incre
        vel_msg = TwistStamped(twist=Twist())  # send zero velocity during imaging

      if self.O_T_EE[0, -1] > x_end:
        print('complete one round')
        break

      self.franka_status_pub.publish(Bool(self.franka_status))
      self.vel_pub.publish(vel_msg)
      self.rate.sleep()

  def leave_entry_pose(self, vert_safe_dist=0.01):
    leave_pose = PoseStamped()
    leave_pose.header.frame_id = 'panda_link0'
    leave_pose_rot = transformations.quaternion_from_matrix(self.O_T_EE)
    leave_pose.pose.orientation.x = leave_pose_rot[0]
    leave_pose.pose.orientation.y = leave_pose_rot[1]
    leave_pose.pose.orientation.z = leave_pose_rot[2]
    leave_pose.pose.orientation.w = leave_pose_rot[3]
    leave_pose.pose.position.x = self.O_T_EE[0, -1]
    leave_pose.pose.position.y = self.O_T_EE[1, -1]
    leave_pose.pose.position.z = self.O_T_EE[2, -1] + vert_safe_dist
    goal = MoveToPoseGoal(goal_pose=leave_pose)
    self.cart_pose_cli.send_goal(goal)
    self.cart_pose_cli.wait_for_result()

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

  def dump_franka_pose(self):
    data = self.O_T_EE.flatten()
    with open(self.robot_logger_path + self.robot_log_file, 'a') as robot_logger:
      writer = csv.writer(robot_logger)
      writer.writerow(data)
      print('pose data dumped to ', self.robot_logger_path + self.robot_log_file)

  def franka_pose_cb(self, msg: FrankaState):
    curr_ee_pos = msg.O_T_EE  # inv 4x4 matrix
    self.O_T_EE = np.array([curr_ee_pos[0:4], curr_ee_pos[4:8],
                            curr_ee_pos[8:12], curr_ee_pos[12:16]]).transpose()

  def pa_status_cb(self, msg: Bool):
    self.pa_status = msg.data


if __name__ == "__main__":
  rospy.init_node('franka_pa_scan')
  # ========== scan params ==========
  scan_count = 0
  scan_vel = 2e-3             # [m/s]
  sample_length = 0.12        # [m]
  sample_width = 0.08         # [m]
  lateral_fov = 0.0125        # [m]
  lateral_overlap = 0.005     # [m]
  num_scan = np.floor(sample_width/(lateral_fov-lateral_overlap))
  lateral_dir = -1  # -y direction
  lateral_increment = lateral_dir*(lateral_fov-lateral_overlap)
  # =================================
  scan = PAScan()

  while not rospy.is_shutdown():
    # scan.go_to_home_config()
    if scan_count > num_scan:
      break
    scan.go_to_entry_pose()
    with np.printoptions(precision=4, suppress=True):
      print('current entry pose: \n', scan.get_entry_pose())
    input('press any key to start new scan round')
    print('scan round: ', scan_count)
    # scan.do_scan_process_blind(scan_path_length=sample_length, scan_vel=scan_vel)
    scan.do_scan_process()
    scan.leave_entry_pose(vert_safe_dist=0.005)
    curr_entry_pose = scan.get_entry_pose()
    curr_entry_pose[1, -1] += lateral_increment
    scan.set_entry_pose(curr_entry_pose)
    scan_count += 1
  print('scan complete')
