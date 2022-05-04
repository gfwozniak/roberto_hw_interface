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
    print("zero auger called")
    zero_thread = threading.Thread(target=robot_functions.zeroAuger)
    zero_thread.start()
    robot_functions.delayinput()
    print("delay input passed!")

    while True:
        if joystick.A:
            thread = threading.Thread(target=robot_functions.noMotorCommand)
            thread.start()
            robot_functions.delayinput()
            #stop motors
        if joystick.B:#out
#            thread = threading.Thread(target=robot_functions.moveBScrew, args=(-2000000,))
#            thread.start()
#            robot_functions.delayinput()
            thread = threading.Thread(target=robot_functions.moveActuator, args=(-50,))
            thread.start()
            robot_functions.delayinput()
            continue
            #j
        if joystick.X:
            print("zero auger called")
            thread = threading.Thread(target=robot_functions.zeroAuger)
            thread.start()
            robot_functions.delayinput()
            print("delay input passed!")
            continue
            #zero auger
        if joystick.Y:#in
#            thread = threading.Thread(target=robot_functions.moveBScrew, args=(-10000,))
#            thread.start()
#            robot_functions.delayinput()
            thread = threading.Thread(target=robot_functions.moveActuator, args=(0,))
            thread.start()
            robot_functions.delayinput()
            continue
            continue
            #deposit
        if joystick.RB:
            thread = threading.Thread(target=robot_functions.moveAuger, args=(.5, 5))
            thread.start()
            robot_functions.delayinput()
            continue
        if joystick.LB:
            thread = threading.Thread(target=robot_functions.moveAuger, args=(-.5, 5))
            thread.start()
            robot_functions.delayinput()
            continue
            #return to driving position
        time.sleep(0.1)
    
    robot_functions.e.set()
    print("finished")

    exit()

    #hold all motors await input to zero actuator and bscrew

    #then all functions open up

