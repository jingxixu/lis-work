#!/usr/bin/env python
'''
----------------------------------------------
-- closed loop demo
-- the robot keeps saying "show me something"
-- if a red block is shown, it says "I do not like red"
-- if a green block is shown, it says "yeah, baby" and then waves arms

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

speak = lambda x: os.system("rosrun sound_play say.py '%s'" % x)
class MotionController:
    def __init__(self):
        self.subscriber = rospy.Subscriber("detected_color", \
            String, self.processString, queue_size=1)
        self.cnt = 0

    def processString(self, string_msg):
        print(string_msg.data)
        if string_msg.data == "red":
            speak("I do not like red")
        elif string_msg.data == "green":
            speak("yeah baby")
            for _ in range(2):
                uc.command_joint_pose('l', [1.5,  -1, 3, 1, 0, 0, 0], time=1, blocking=True)
                uc.command_joint_pose('r', [-1.5, -1,-3, 1, 0, 0, 0], time=1, blocking=True)
                time.sleep(1)
                uc.command_joint_pose('l', [1.5,  0.3, 3, 0, 0, 0, 0], time=1, blocking=True)
                uc.command_joint_pose('r', [-1.5, 0.3,-3, 0, 0, 0, 0], time=1, blocking=True)
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
    uc.command_joint_pose('l', [1.5,  0.3, 3, 0, 0, 0, 0], time=1, blocking=True)
    uc.command_joint_pose('r', [-1.5, 0.3,-3, 0, 0, 0, 0], time=1, blocking=True)

    detector = Detector()
    controller = MotionController()
    rospy.spin()