#!/usr/bin/env python

import rospy
from robot_api import RobertoAPI
from robot_api import JoystickReader

def Manual(roberto, joystick):
    while not rospy.is_shutdown():
        roberto._drivetrain_linear_x_cmd_ = joystick.linearx
        roberto._drivetrain_angular_z_cmd_ = joystick.angularz

if __name__ == '__main__':
    try:
        rospy.init_node('auger_motor_calls', anonymous=True)
        robot = RobertoAPI()
        joy = JoystickReader()
        Manual(robot, joy)
    except rospy.ROSInterruptException:
        pass