#!/usr/bin/env python

import rospy
from auger_motor_api import AugerAPI

if __name__ == '__main__':
    try:
        rospy.init_node('auger_motor_calls', anonymous=True)
        robot = AugerAPI()
        robot.setAugerVelocityForDuration(velocity=100, seconds=10)
    except rospy.ROSInterruptException:
        pass