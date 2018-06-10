#!/usr/bin/env python
'''
----------------------------------------------
-- same closed loop demo as reactive_jingxi.py
-- but directly uses a simple action client to control the movement of arms
-- instead of using the functions from uber controller to do so

-- Jingxi Xu
----------------------------------------------
'''

from lis_pr2_pkg.uber_controller import Uber
import rospy
import os
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import time
from actionlib import SimpleActionClient as SAC
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus as gs


class Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/head_mount_kinect/rgb/image_rect_color", \
            Image, self.processImage, queue_size=1)
        # publish the color detected "red" or "green"
        self.publisher_color = rospy.Publisher("detected_color", String, queue_size=1)
        rospy.loginfo("Detector initialized.")

    def processImage(self, image_msg):
        image_bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
        # equivalent to 
        # >>> desired_encoding='bgr8' or 'bgra8'
        image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        lower_green = np.array([40,100,100])
        upper_green = np.array([70,255,255])

        lower_red1 = np.array([160, 100, 100])
        upper_red1 = np.array([180, 255, 255])

        lower_red2 = np.array([0, 100, 100])
        upper_red2 = np.array([10, 255, 255])

        mask_red = cv2.inRange(image_hsv, lower_red1, upper_red1) | cv2.inRange(image_hsv, lower_red2, upper_red2)
        mask_green = cv2.inRange(image_hsv, lower_green, upper_green)
        res = cv2.bitwise_and(image_bgr, image_bgr, mask= mask_green)

        # print "red: ", np.sum(mask_red)
        # print "green: ", np.sum(mask_green)
        if np.sum(mask_red) > 3000000:
            self.publisher_color.publish("red")
        # green is a bit hard to detect
        elif np.sum(mask_green) > 2000000: 
            self.publisher_color.publish("green")
        else:
            self.publisher_color.publish("none red or green")
        # cv2.imshow('image',image_bgr)
        # cv2.waitKey(0)
        # cv2.imshow('image',mask_green)
        # cv2.waitKey(0)
        # cv2.imshow('image',mask_red)
        # cv2.waitKey(0)
        # cv2.imshow('image',res)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

## action client for arm movement
class Arm:
    def __init__(self, arm):
        if arm == 'r':
            self.arm_client = SAC('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
            self.arm = arm
        elif arm == 'l':
            self.arm_client = SAC('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
            self.arm = arm

        self.arm_client.wait_for_server()
        rospy.loginfo('Waiting for server...')

    def move(self, angles, time, blocking):
        angles = [angles]
        times = [time]
        timeout=times[-1] + 1.0

        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names =self.get_joint_names(self.arm)
        
        for (ang, t) in zip(angles, times):
            point = JointTrajectoryPoint()
            point.positions = ang
            point.time_from_start = rospy.Duration(t)
            goal.trajectory.points.append(point)
        goal.trajectory.header.stamp = rospy.Time.now()
        
        self.arm_client.send_goal(goal)
        rospy.sleep(.1)
        rospy.loginfo("command sent to client")
        status = 0

        if blocking: #XXX why isn't this perfect?
            end_time = rospy.Time.now() + rospy.Duration(timeout+ .1)
            while (
                    (not rospy.is_shutdown()) and\
                    (rospy.Time.now() < end_time) and\
                    (status < gs.SUCCEEDED) and\
                    (type(self.arm_client.action_client.last_status_msg) != type(None))):
                status = self.arm_client.action_client.last_status_msg.status_list[-1].status #XXX get to 80
                rospy.Rate(10).sleep()
            if status >gs.SUCCEEDED:
                rospy.loginfo("goal status achieved.  exiting")
            else:
                rospy.loginfo("ending due to timeout")

        result = self.arm_client.get_result()
        return result

    def  get_joint_names(self, char):
        return [char+'_shoulder_pan_joint',
                char+'_shoulder_lift_joint',
                char+'_upper_arm_roll_joint',
                char+'_elbow_flex_joint',
                char+'_forearm_roll_joint',
                char+'_wrist_flex_joint',
                char+'_wrist_roll_joint' ]

speak = lambda x: os.system("rosrun sound_play say.py '%s'" % x)
class MotionController:
    def __init__(self):
        self.subscriber = rospy.Subscriber("detected_color", \
            String, self.processString, queue_size=1)
        self.cnt = 0
        self.left_arm = Arm('l')
        self.right_arm = Arm('r')

    def processString(self, string_msg):
        print(string_msg.data)
        if string_msg.data == "red":
            speak("I do not like red")
        elif string_msg.data == "green":
            speak("yeah baby")
            for _ in range(2):
                self.left_arm.move([1.5,  -1, 3, 1, 0, 0, 0], time=1, blocking=False)
                self.right_arm.move([-1.5, -1,-3, 1, 0, 0, 0], time=1, blocking=False)
                time.sleep(1)
                self.left_arm.move([1.5,  0.3, 3, 0, 0, 0, 0], time=1, blocking=False)
                self.right_arm.move([-1.5, 0.3,-3, 0, 0, 0, 0], time=1, blocking=False)
                time.sleep(1)
        else:
            speak("show me something")
        self.cnt+=1
        if self.cnt == 10:
            self.subscriber.unregister()

if __name__ == "__main__":
    rospy.init_node("corlor_detector")
    rospy.loginfo('Node initialized. Yeah, Baby!')

    uc = Uber()
    uc.command_head([0, 1], time=1, blocking=False)
    time.sleep(1)
    uc.close_gripper('l')
    uc.close_gripper('r')
    # whether blocking is true really does not matter here
    uc.command_joint_pose('l', [1.5,  0.3, 3, 0, 0, 0, 0], time=1, blocking=True)
    uc.command_joint_pose('r', [-1.5, 0.3,-3, 0, 0, 0, 0], time=1, blocking=True)

    detector = Detector()
    controller = MotionController()
    rospy.spin()