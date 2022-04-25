#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class SafeRobot:

    def __init__(self):
        self._pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('PythonSafetyLayer', anonymous=True)
        self._rate = rospy.Rate(10) # 10hz

    def publish(self):
        while not rospy.is_shutdown():
            hello_str = "hello world %s" % rospy.get_time()
            rospy.loginfo(hello_str)
            self._pub.publish(hello_str)
            self._rate.sleep()

