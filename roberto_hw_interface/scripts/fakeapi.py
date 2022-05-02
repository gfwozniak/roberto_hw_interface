#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import threading

class AugerAPI:

#
# INITIALIZE OBJECT
#
    def __init__(self):
        # INITIALIZE PUBLISHERS
        self._actuator_publisher = rospy.Publisher('actuator_cmd', Float64, queue_size=1)
        self._bscrew_publisher = rospy.Publisher('bscrew_cmd', Float64, queue_size=1)
        self._auger_publisher = rospy.Publisher('auger_cmd', Float64, queue_size=1)

        # INITIALIZE SUBSCRIBERS and VARIABLES TO STORE DATA
        self.actuator_position = 0.0
        self.bscrew_position = 0.0
        self.is_moving = False

        # PARAMS
        self.actuator_range = rospy.get_param("~actuator_range")
        self.bscrew_range = rospy.get_param("~bscrew_range")

        #START HOSTING NODE
        self._rate = rospy.Rate(10) # 10hz

    def printparam(self):
        print(self.actuator_range)

