#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import time

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
        rospy.Subscriber('actuator_pos', Float64, self._actuator_callback)
        rospy.Subscriber('bscrew_pos', Float64, self._actuator_callback)

#        # PARAMS
#        self.actuator_range = rospy.get_param("~actuator_range")
#        self.bscrew_range = rospy.get_param("~bscrew_range")

        # OTHER VARIABLES
        self._rate = rospy.Rate(10) # 10hz
        self._actuator_error = 2.0;
        self._bscrew_error = 5000.0;

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
# HELPER METHODS
#
#
#    def _is_driveable(self):
#        return True
#        #to do
#

#
# MOTOR OPERATION METHODS
#
    def setAugerVelocityForDuration(self, velocity, seconds):
        start_time = time.time()
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time > seconds:
                break
            self._auger_publisher.publish(Float64(velocity))
            self._rate.sleep()

    def setBScrewPosition(self, position):
        while not rospy.is_shutdown():
            if ((self.bscrew_position < (position + self._bscrew_error)) and (self.bscrew_position > (position - self._bscrew_error))):
                break
            self._bscrew_publisher.publish(Float64(position))
            self._rate.sleep()
        
    def setBScrewPositionAndAugerVelocity(self, position, velocity):
        while not rospy.is_shutdown():
            if ((self.bscrew_position < (position + self._bscrew_error)) and (self.bscrew_position > (position - self._bscrew_error))):
                break
            self._bscrew_publisher.publish(Float64(position))
            self._auger_publisher.publish(Float64(velocity))
            self._rate.sleep()
        self._auger_publisher.publish(Float64(0))

    def setActuatorPosition(self, position):
        while not rospy.is_shutdown():
            if ((self.actuator_position < (position + self._actuator_error)) and (self.actuator_position > (position - self._actuator_error))):
                break
            self._actuator_publisher.publish(Float64(position))
            self._rate.sleep()

    def runBScrewToLimit():
        pass
        
    def stopMotors(self):
        pass