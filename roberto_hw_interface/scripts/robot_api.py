#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import time

class RobertoAPI:

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
        self.limit_switch_position = 0
        self.is_moving = False
        rospy.Subscriber('actuator_pos', Float64, self._actuator_callback)
        rospy.Subscriber('bscrew_pos', Float64, self._bscrew_callback)
        rospy.Subscriber('limit_switch', Bool, self._limit_switch_callback)

#        # PARAMS
#        self.actuator_range = rospy.get_param("~actuator_range")
#        self.bscrew_range = rospy.get_param("~bscrew_range")

        # OTHER VARIABLES
        self._rate = rospy.Rate(10) # 10hz
        self._actuator_error = 2.0;
        self._bscrew_error = 100000.0;

#
# CALLBACK METHODS for SUBSCRIBERS
#
    def _actuator_callback(self, value):
        self.actuator_position = value.data

    def _bscrew_callback(self, value):
        self.bscrew_position = value.data

    def _limit_switch_callback(self, value):
        self.limit_switch_position = value.data

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
        print("set auger velocity")
        start_time = time.time()
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time > seconds:
                break
            self._auger_publisher.publish(Float64(velocity))
            self._rate.sleep()

    def setBScrewPosition(self, position):
        print("set bscrew position")
        while not rospy.is_shutdown():
            if ((self.bscrew_position < (position + self._bscrew_error)) and (self.bscrew_position > (position - self._bscrew_error))):
                break
            self._bscrew_publisher.publish(Float64(position))
            self._rate.sleep()
        
    def setBScrewPositionAndAugerVelocity(self, position, velocity):
        print("setting bscrew position with auger speed")
        while not rospy.is_shutdown():
            if ((self.bscrew_position < (position + self._bscrew_error)) and (self.bscrew_position > (position - self._bscrew_error))):
                print("good")
                break
            self._bscrew_publisher.publish(Float64(position))
            self._auger_publisher.publish(Float64(velocity))
            self._rate.sleep()
        self._auger_publisher.publish(Float64(0))

    def setActuatorPositionTimeout(self, position, seconds):
        print("setting actuator position")
        start_time = time.time()
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time > seconds:
                break
            if ((self.actuator_position < (position + self._actuator_error)) and (self.actuator_position > (position - self._actuator_error))):
                break
            omsg = Float64()
            omsg.data = position
            self._actuator_publisher.publish(omsg)
            self._rate.sleep()

    def zeroBScrew(self):
        print("zeroing bscrew")
        while not rospy.is_shutdown():
            if (self.limit_switch_position):
                break
            self._bscrew_publisher.publish(Float64(8000000))
            self._rate.sleep()
        self._bscrew_publisher.publish(Float64(0))
        
    def stopMotors(self):
        pass