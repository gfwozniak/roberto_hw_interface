#!/usr/bin/env python

from robot_functions import RobertoFunctions
from robot_api import RobertoAPI
from robot_api import JoystickReader
import threading
import time
import rospy

class Interrupter:

    def __init__(self):
        self.e = threading.Event()

    def interrupt(self):
        self.e.set()
        while True:
            if (not self.e.is_set()):
                break
            time.sleep(.25)



if __name__ == '__main__':
    rospy.init_node('Joy2RobotControl')
    robot_functions = RobertoFunctions()
    joystick = JoystickReader()

    init = threading.Thread(target=robot_functions.initialMotors)
    init.start()

    joystick.waitUntilX(timeout=100, period=.25)
    print("Initializing motors")
    initialZero = threading.Thread(target=robot_functions.zeroAuger)
    initialZero.start()

    while True:
        if joystick.A:
            pass
            #stop motors
        if joystick.B:
            pass
            #j
        if joystick.X:
            pass
            #zero auger
        if joystick.Y:
            pass
            #deposit
        if joystick.RB:
            pass
            #return to driving position
        if joystick.LB:
            pass
            #return to driving position
        

        initialZero.join()
    
    print("finished")

    #hold all motors await input to zero actuator and bscrew

    #then all functions open up

