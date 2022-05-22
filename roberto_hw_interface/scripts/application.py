#!/usr/bin/env python

from robot_functions import RobertoFunctions
from robot_api import JoystickReader
import threading
import time
import rospy

class Application:
    def __init__(self):
        self.robot_functions = RobertoFunctions()
        self.joystick = JoystickReader()
        
    def execute(self):
        init_thread = threading.Thread(target=self.robot_functions.initialMotors)
        init_thread.start()

        # START APPLICATION ON XBOX PRESS
        print("WAITING FOR SQUARE, PRESS A AT ANY TIME TO STOP MOTORS")
        self.joystick.waitUntilSQUARE(timeout=1000000, period=.1)
        print("SQUARE PRESSED: ZEROING AUGER")
        zero_thread = threading.Thread(target=self.robot_functions.zeroAuger)
        zero_thread.start()
        self.robot_functions.delayinput()

        rospy.Timer(rospy.Duration(0.05), self.driveLoop)

        while True:
            if self.joystick.SQUARE: # zero auger with XBOX
                print("zero auger called")
                thread = threading.Thread(target=self.robot_functions.zeroAuger)
                thread.start()
                self.robot_functions.delayinput()
                print("delay input passed!")
                continue

            if self.joystick.A: # stop motors with A
                print('a')
                thread = threading.Thread(target=self.robot_functions.noMotorCommand)
                thread.start()
                self.robot_functions.delayinput()

            if self.joystick.B: # neutral position with B
                thread = threading.Thread(target=self.robot_functions.neutralPosition)
                thread.start()
                self.robot_functions.delayinput()
                continue

            if self.joystick.X: # mine with X
                #thread = threading.Thread(target=self.robot_functions.mine)
                thread = threading.Thread(target=self.robot_functions.moveBScrewFast, args=(0,))
                thread.start()
                self.robot_functions.delayinput()
                continue

            if self.joystick.Y: # deposit with Y
                #thread = threading.Thread(target=self.robot_functions.deposit)
                thread = threading.Thread(target=self.robot_functions.moveBScrewSlow, args=(2000000,))
                thread.start()
                self.robot_functions.delayinput()
                continue

            time.sleep(0.1)
        
        self.robot_functions.e.set()

    def driveLoop(self, event):
        self.robot_functions.robot_api._drivetrain_linear_x_cmd_ = self.joystick.linearx * 0.3
        self.robot_functions.robot_api._drivetrain_angular_z_cmd_ = -self.joystick.angularz * 0.3

if __name__ == '__main__':
    rospy.init_node('Joy2RobotControl')
    application = Application()
    application.execute()



#    # INITIALIZE ROS
#    rospy.init_node('Joy2RobotControl')
#    robot_functions = RobertoFunctions()
#    joystick = JoystickReader()
#
#    # INITIALIZE MOTOR COMMANDS
#    init_thread = threading.Thread(target=robot_functions.initialMotors)
#    init_thread.start()
#
#    # START APPLICATION ON XBOX PRESS
#    print("WAITING FOR XBOX, PRESS A AT ANY TIME TO STOP MOTORS")
#    joystick.waitUntilXBOX(timeout=100, period=.1)
#    print("XBOX PRESSED: ZEROING AUGER")
#    zero_thread = threading.Thread(target=robot_functions.zeroAuger)
#    zero_thread.start()
#    robot_functions.delayinput()
#
#    #rospy.Timer(rospy.Duration(0.05), self.controlLoop)
#
#    while True:
#        if joystick.XBOX: # zero auger with XBOX
#            print("zero auger called")
#            thread = threading.Thread(target=robot_functions.zeroAuger)
#            thread.start()
#            robot_functions.delayinput()
#            print("delay input passed!")
#            continue
#
#        if joystick.A: # stop motors with A
#            thread = threading.Thread(target=robot_functions.noMotorCommand)
#            thread.start()
#            robot_functions.delayinput()
#
#        if joystick.B: # neutral position with B
#            thread = threading.Thread(target=robot_functions.neutralPosition)
#            thread.start()
#            robot_functions.delayinput()
#            continue
#
#        if joystick.X: # mine with X
#            thread = threading.Thread(target=robot_functions.mine)
#            thread.start()
#            robot_functions.delayinput()
#            continue
#
#        if joystick.Y: # deposit with Y
#            thread = threading.Thread(target=robot_functions.deposit)
#            thread.start()
#            robot_functions.delayinput()
#            continue
#
#        time.sleep(0.1)
#    

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
