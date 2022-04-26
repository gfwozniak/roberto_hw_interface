#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
import time

class SafeRobot:

    def __init__(self):
        self._pub2 = rospy.Publisher('tter', String, queue_size=10)
        self._pub3 = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('PythonSafetyLayer', anonymous=True)
        self._rate = rospy.Rate(10) # 10hz

    def publish2(self, seconds):
        start_time = time.time()
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time > seconds:
                print("Finished iterating in: " + str(int(elapsed_time))  + " seconds")
                break
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self._pub2.publish(hello_str)
            self._rate.sleep()

    def publish3(self, seconds):
        start_time = time.time()
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time > seconds:
                print("Finished iterating in: " + str(int(elapsed_time))  + " seconds")
                break
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self._pub3.publish(hello_str)
            self._rate.sleep()