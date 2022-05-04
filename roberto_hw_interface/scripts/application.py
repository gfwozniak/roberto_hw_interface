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

    print("waiting for xbox")
    joystick.waitUntilXBOX(timeout=100, period=.1)
    print("xbox pressed")
    print("zero auger called")
    zero_thread = threading.Thread(target=robot_functions.zeroAuger)
    zero_thread.start()
    robot_functions.delayinput()

    while True:
        if joystick.XBOX: # zero auger with XBOX
            print("zero auger called")
            thread = threading.Thread(target=robot_functions.zeroAuger)
            thread.start()
            robot_functions.delayinput()
            print("delay input passed!")
            continue

        if joystick.A: # stop motors with A
            thread = threading.Thread(target=robot_functions.noMotorCommand)
            thread.start()
            robot_functions.delayinput()

        if joystick.B: # neutral position with B
            thread = threading.Thread(target=robot_functions.neutralPosition)
            thread.start()
            robot_functions.delayinput()
            continue

        if joystick.X: # mine with X
            thread = threading.Thread(target=robot_functions.mine)
            thread.start()
            robot_functions.delayinput()
            continue

        if joystick.Y: # deposit with Y
            thread = threading.Thread(target=robot_functions.deposit)
            thread.start()
            robot_functions.delayinput()
            continue

        if joystick.RB:
            continue

        if joystick.LB:
            continue

        time.sleep(0.1)
    
    robot_functions.e.set()
    print("finished")

    exit()

    # MOVE BSCREW
#            thread = threading.Thread(target=robot_functions.moveBScrew, args=(-10000,))
#            thread.start()
#            robot_functions.delayinput()

    # MOVE ACTUATOR
#            thread = threading.Thread(target=robot_functions.moveActuator, args=(-5,))
#            thread.start()
#            robot_functions.delayinput()

    # MOVE AUGER
#            thread = threading.Thread(target=robot_functions.moveAuger, args=(.5, 5))
#            thread.start()
#            robot_functions.delayinput()
