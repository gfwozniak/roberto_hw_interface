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

    init_thread = threading.Thread(target=robot_functions.initialMotors)
    init_thread.start()

    print("waiting for x")
    joystick.waitUntilX(timeout=100, period=.1)
    print("x pressed")
    zero_thread = threading.Thread(target=robot_functions.zeroAuger)
    zero_thread.start()
    robot_functions.delayinput()

    while True:
        if joystick.A:
            thread = threading.Thread(target=robot_functions.noMotorCommand)
            thread.start()
            robot_functions.delayinput()
            #stop motors
        if joystick.B:
            continue
            #j
        if joystick.X:
            thread = threading.Thread(target=robot_functions.zeroAuger)
            thread.start()
            robot_functions.delayinput()
            continue
            #zero auger
        if joystick.Y:
            thread = threading.Thread(target=robot_functions.deposit)
            thread.start()
            robot_functions.delayinput()
            continue
            #deposit
        if joystick.RB:
            thread = threading.Thread(target=robot_functions.returnToNeutral)
            thread.start()
            robot_functions.delayinput()
            #return to driving position
        if joystick.LB:
            break
            continue
            #return to driving position
        time.sleep(0.1)
    
    robot_functions.e.set()
    print("finished")

    exit()

    #hold all motors await input to zero actuator and bscrew

    #then all functions open up

