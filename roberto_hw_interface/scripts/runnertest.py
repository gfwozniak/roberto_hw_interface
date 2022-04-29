#!/usr/bin/env python

import rospy
from auger_motor_api import AugerAPI

if __name__ == '__main__':
    try:
        rospy.init_node('auger_motor_calls', anonymous=True)
        robot = AugerAPI()
        print("zero bscrew")
        robot.zeroBScrew()
        print("finished")
    except rospy.ROSInterruptException:
        pass