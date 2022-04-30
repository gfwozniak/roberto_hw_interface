#!/usr/bin/env python

import rospy
from auger_functions import AugerFunctions

if __name__ == '__main__':
    try:
        rospy.init_node('auger_motor_calls', anonymous=True)
        robot = AugerFunctions()
        print("zero bscrew")
        robot.zeroAuger()
        print("finished")
    except rospy.ROSInterruptException:
        pass