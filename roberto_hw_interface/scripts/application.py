#!/usr/bin/env python

from robot_functions import RobertoFunctions
from robot_api import RobertoAPI
from robot_api import JoystickReader
import threading
import time

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

    robot_functions = RobertoFunctions()
    joystick = JoystickReader()

    initialPosition = threading.Thread(target=robot_functions.initialMotors())
    initialPosition.start()

    joystick.waitUntilX(timeout=100, period=.25)
    initialZero = threading.Thread(target=robot_functions.zeroAuger())
    
    print("finished")

    #hold all motors await input to zero actuator and bscrew

    #then all functions open up

