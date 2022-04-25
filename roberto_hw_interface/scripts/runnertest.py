#!/usr/bin/env python

import rospy
from pythonrobotapi import SafeRobot

if __name__ == '__main__':
    try:
        robot = SafeRobot()
        robot.publish()
    except rospy.ROSInterruptException:
        pass