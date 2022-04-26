#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Roberto:

#
# INITIALIZE OBJECT
#
    def __init__(self):
        # INITIALIZE PUBLISHERS
        self._drivetrain_publisher = rospy.Publisher('drivetrain_cmd', Twist, queue_size=1)
        self._actuator_publisher = rospy.Publisher('actuator_cmd', Float64, queue_size=1)
        self._bscrew_publisher = rospy.Publisher('bscrew_cmd', Float64, queue_size=1)
        self._auger_publisher = rospy.Publisher('auger_cmd', Float64, queue_size=1)

        # INITIALIZE SUBSCRIBERS and VARIABLES TO STORE DATA
        self.actuator_position = 0.0
        self.bscrew_position = 0.0
        self.is_moving = False
        rospy.Subscriber('actuator_pos', Float64, self._actuator_callback)

        #START HOSTING NODE
        rospy.init_node('PythonFunctionHandler', anonymous=True)
        self._rate = rospy.Rate(10) # 10hz

#
# CALLBACK METHODS for SUBSCRIBERS
#
    def _actuator_callback(self, value):
        self.actuator_position = value.data
        rospy.loginfo(self.actuator_position)

    def _bscrew_callback(self, value):
        self.bscrew_position = value.data

#    def _actuator_callback(self, value):
#        self.is_moving = value.data

#
# MOTOR OPERATION METHODS
#
    def setDrivetrainVelocity(self, linear, angular):
        while not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self._drivetrain_publisher.publish(msg)
            self._rate.sleep()

    def setAugerVelocity(self, velocity):
        while not rospy.is_shutdown():
            self._auger_publisher.publish(Float64(velocity))
            self._rate.sleep()

    def setBScrewPosition(self, position):
        while not rospy.is_shutdown():
            self._bscrew_publisher.publish(Float64(position))
            self._rate.sleep()
        
    def setActuatorPosition(self, position):
        while not rospy.is_shutdown():
            self._actuator_publisher.publish(Float64(position))
            self._rate.sleep()
        