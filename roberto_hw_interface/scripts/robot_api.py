#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
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
        self._drivetrain_publisher = rospy.Publisher('drivetrain_cmd', Twist, queue_size=1)

        # INITIALIZE COMMAND VARIABLES
        self._drivetrain_linear_x_cmd_ = 0.0
        self._drivetrain_angular_z_cmd_ = 0.0
        self._actuator_position_cmd_ = 0.0
        self._bscrew_position_cmd_ = 0.0
        self._auger_run_cmd_ = 0.0

        # INITIALIZE SERVICE CALLS
        rospy.wait_for_service('zero_bscrew')
        self.zeroBScrew = rospy.ServiceProxy('zero_bscrew', Empty)
        rospy.wait_for_service('zero_actuator')
        self.zeroActuator = rospy.ServiceProxy('zero_actuator', Empty)

        # INITIALIZE SUBSCRIBERS and VARIABLES TO STORE DATA
        rospy.Subscriber('joint_states', JointState, self._joint_state_callback)
        rospy.Subscriber('bscrew_limit', Bool, self._limit_switch_callback)

        # [1] actuator
        # [2] auger
        # [3] bscrew
        # [4] wheel 0
        # [5] wheel 1

        self.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.effort = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.limit_switch_position = False

#        # PARAMS
#        self.actuator_range = rospy.get_param("~actuator_range")
#        self.bscrew_range = rospy.get_param("~bscrew_range")

        # OTHER VARIABLES
        self._rate = rospy.Rate(10) # 10hz
        self._actuator_error = 1.0
        self._bscrew_error = 100000.0

        # TIMER
        rospy.Timer(rospy.Duration(0.1), self.controlLoop)

    def controlLoop(self, event):
        while not rospy.is_shutdown():
            self._auger_publisher.publish(Float64(self._auger_run_cmd_))
            self._bscrew_publisher.publish(Float64(self._bscrew_position_cmd_))
            self._actuator_publisher.publish(Float64(self._actuator_position_cmd_))
            twist = Twist()
            twist.linear.x = (self._drivetrain_linear_x_cmd_)
            twist.angular.z = (self._drivetrain_angular_z_cmd_)
            self._drivetrain_publisher.publish(twist)
            self._rate.sleep()
#
# CALLBACK METHODS for SUBSCRIBERS
#
    def _joint_state_callback(self, message):
        pass

    def _limit_switch_callback(self, message):
        self.limit_switch_position = message.data

#
# OPERATION METHODS
# 
    def setAugerVelocity(self, velocity):
        self._auger_run_cmd_ = velocity

    def setBScrewPosition(self, position):
        self._bscrew_position_cmd_ = position

    def setActuatorPosition(self, position):
        self._actuator_position_cmd_ = position

    def setDrivetrainVelocity(self, linearvelocity, angularvelocity):
        self._drivetrain_angular_z_cmd_ = angularvelocity
        self._drivetrain_linear_x_cmd_ = linearvelocity


    def waitUntilActuatorPosition(self, timeout, period, targetpos):
        mintarget = targetpos - self._actuator_error
        maxtarget = targetpos + self._actuator_error
        mustend = time.time() + timeout
        while time.time() < mustend:
            if (self.position[1] > mintarget and self.position[1] < maxtarget): 
                return True
            time.sleep(period)
        return False

    def waitUntilBScrewPosition(self, timeout, period, targetpos):
        mintarget = targetpos - self._bscrew_error
        maxtarget = targetpos + self._bscrew_error
        mustend = time.time() + timeout
        while time.time() < mustend:
            if (self.position[1] > mintarget and self.position[1] < maxtarget): 
                return True
            time.sleep(period)
        return False
    
    def waitUntilLimit(self, timeout, period):
        mustend = time.time() + timeout
        while time.time() < mustend:
            if (self.limit_switch_position): 
                return True
            time.sleep(period)
        return False
    
class JoystickReader:

    def __init__(self):
        self.linearx = 0.0
        self.angularz = 0.0
        self.A = False
        self.B = False
        self.Y = False
        self.X = False
        self.LB = False
        self.RB = False
        rospy.Subscriber("joy", Joy, self._joystick_callback)
        
    def _combineLTRT(self, message):
        LT = -(message.axes[2] + 1.0) / 2
        RT = -(message.axes[5] + 1.0) / 2
        return (RT - LT)
    
    def _joystick_callback(self, message):
        self.linearx = self._combineLTRT(message)
        self.angularz = message.axes[0]
        self.A = message.buttons[0]
        self.B = message.buttons[1]
        self.X = message.buttons[2]
        self.Y = message.buttons[3]
        self.LB = message.buttons[4]
        self.RB = message.buttons[5]

    def waitUntilX(self, timeout, period):
        mustend = time.time() + timeout
        while time.time() < mustend:
            if self.X: 
                return True
            time.sleep(period)
        return False
