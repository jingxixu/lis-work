#!/usr/bin/env python
'''
----------------------------------------------
-- dance demo to play around with robot joints
   and pose configurations, various motions,
   etc.

-- Jingxi Xu
----------------------------------------------
'''

from lis_pr2_pkg.uber_controller import Uber
import rospy
import numpy as np
import time 
import csv

rospy.init_node("ubertest")
rospy.loginfo("Jingxi's dance demo")
uc = Uber()

# initial
uc.close_gripper('l')
uc.close_gripper('r')
uc.command_joint_pose('l', [0]*7, time=2, blocking=True)
uc.command_joint_pose('r', [0]*7, time=2, blocking=True)
uc.look_forward()

# dance 1
for i in range(2):
  l = [ 0.95, 0.0, np.pi/2., -2, -np.pi*1.5, 0, np.pi]
  r = [ -0.95, 0.0, -np.pi/2., 0, np.pi*1.5, 0, np.pi]
  uc.command_head([-1, 0], time=1, blocking=False)
  uc.command_joint_pose('l', l, time=1, blocking=True)
  uc.command_joint_pose('r', r, time=1, blocking=True)
  time.sleep(1)
  uc.command_head([0, 0], time=1, blocking=False)
  uc.command_joint_pose('l', [0]*7, time=1, blocking=True)
  uc.command_joint_pose('r', [0]*7, time=1, blocking=True)
  time.sleep(1)

  l = [ 0.95, 0.0, np.pi/2., 0, -np.pi*1.5, 0, np.pi]
  r = [ -0.95, 0.0, -np.pi/2., -2, np.pi*1.5, 0, np.pi]
  uc.command_head([1, 0], time=1, blocking=False)
  uc.command_joint_pose('l', l, time=1, blocking=True)
  uc.command_joint_pose('r', r, time=1, blocking=True)
  time.sleep(1)
  uc.command_head([0, 0], time=1, blocking=False)
  uc.command_joint_pose('l', [0]*7, time=1, blocking=True)
  uc.command_joint_pose('r', [0]*7, time=1, blocking=True)
  time.sleep(1)

# dance 2
for i in range(2):

  uc.down_torso()
  uc.command_head([0, 1], time=1, blocking=False)
  uc.command_joint_pose('l', [1.5,  0.3, 3, 0, 0, 0, 0], time=1, blocking=True)
  uc.command_joint_pose('r', [-1.5, 0.3,-3, 0, 0, 0, 0], time=1, blocking=True)
  time.sleep(1)
  uc.lift_torso()
  uc.command_head([0, -1], time=1, blocking=False)
  uc.command_joint_pose('l', [1.5,  -1, 3, -1, 0, -1, 0], time=2, blocking=True)
  uc.command_joint_pose('r', [-1.5, -1,-3, -1, 0, -1, 0], time=2, blocking=True)
  time.sleep(2)
  uc.command_joint_pose('l', [1.5,  -1, 3, 1, 0, 0, 0], time=1, blocking=True)
  uc.command_joint_pose('r', [-1.5, -1,-3, 1, 0, 0, 0], time=1, blocking=True)
  time.sleep(1)

# final
uc.look_forward()
uc.move_arm_to_side("l")
uc.move_arm_to_side("r")
