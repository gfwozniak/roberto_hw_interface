#!/usr/bin/env python

from robot_api import RobertoAPI
import threading
import time

class RobertoFunctions:

    def __init__(self):
        self.robot_api = RobertoAPI()
        self.e = threading.Event()

    def interrupt(self):
        self.e.set()
        while True:
            if (not self.e.is_set()):
                break
            time.sleep(.25)

    def initialMotors(self):
        print("start initial motors")
        self.robot_api.setAugerVelocity(0.0)
        self.robot_api.setDrivetrainVelocity(0.0, 0.0)
        self.robot_api.setBScrewPosition(self.robot_api.position[3])
        self.robot_api.setActuatorPosition(self.robot_api.position[1])
        self.waitUntilEvent()
        print("end initial motors")

    def stopMotors(self):
        self.interrupt()
        self.robot_api.setAugerVelocity(0.0)
        self.robot_api.setDrivetrainVelocity(0.0, 0.0)
        self.robot_api.setBScrewPosition(self.robot_api.position[3])
        self.robot_api.setActuatorPosition(self.robot_api.position[1])
        self.waitUntilEvent()

    def zeroBScrew(self):
        print("start zeroBscrew")
        self.interrupt()
        self.robot_api.setBScrewPosition(8000000)
        self.waitUntilLimit()
        self.robot_api.setBScrewPosition(0)
        self.waitUntilEvent()
        print("end zeroBscrew")

    def zeroAuger(self):
        print("start zeroBscrew")
        self.interrupt()
        self.robot_api.setBScrewPosition(8000000)
        self.waitUntilLimit()
        self.robot_api.setBScrewPosition(0)
        self.robot_api.setActuatorPosition(10000)
        time.sleep(10)
        self.robot_api.zeroActuator()
        self.robot_api.setActuatorPosition(0)
        self.waitUntilEvent()

#    def waitUntilActuatorPosition(self, timeout, period, targetpos):
#        mintarget = targetpos - self._actuator_error
#        maxtarget = targetpos + self._actuator_error
#        mustend = time.time() + timeout
#        while time.time() < mustend:
#            if (self.position[1] > mintarget and self.position[1] < maxtarget): 
#                return True
#            time.sleep(period)
#        return False
#
#    def waitUntilBScrewPosition(self, timeout, period, targetpos):
#        mintarget = targetpos - self._bscrew_error
#        maxtarget = targetpos + self._bscrew_error
#        mustend = time.time() + timeout
#        while time.time() < mustend:
#            if (self.position[1] > mintarget and self.position[1] < maxtarget): 
#                return True
#            time.sleep(period)
#        return False
    
    def waitUntilLimit(self):
        mustend = time.time() + 100
        while time.time() < mustend:
            if (self.robot_api.limit_switch_position): 
                return True
            if (self.e.is_set()):
                self.e.clear()
                return False
            time.sleep(.5)
        return False

    def waitUntilEvent(self):
        while True:
            if (self.e.is_set()):
                self.e.clear()
                break
            time.sleep(.5)

#    def __init__(self):
#        self.augerapi = RobertoAPI()
#        self.isRunning = False
#
#    def returntoNeutral(self):
#        self.isRunning = True
#        self.augerapi.setBScrewPosition(position=-5000)
#        self.augerapi.setActuatorPositionTimeout(position=1010, seconds=10)
#        self.isRunning = False
#
#    def deployAuger(self):
#        self.isRunning = True
#        if (self.augerapi.bscrew_position > 20000 or self.augerapi.bscrew_position < -20000):
#            pass
#        self.augerapi.setActuatorPositionTimeout(position=940, seconds=15)
#        self.augerapi.setBScrewPositionAndAugerVelocity(position=-7000000, velocity=0.5)
#        self.returnToNeutral()
#        self.isRunning = False
#
#    def depositAuger(self):
#        self.isRunning = True
#        if (self.augerapi.bscrew_position > 20000 or self.augerapi.bscrew_position < -20000):
#            pass
#        self.augerapi.setActuatorPositionTimeout(position=1040, seconds=15)
#        self.augerapi.setBScrewPosition(position=-2000000)
#        self.augerapi.setAugerVelocityForDuration(velocity=-.8, seconds=6)
#        self.augerapi.setAugerVelocityForDuration(velocity=0, seconds=1)
#        self.returnToNeutral()
#        self.isRunning = False
#
#    def zeroAuger(self):
#        self.isRunning = True
#        self.augerapi.zeroBScrew()
#        self.returnToNeutral()
#        self.isRunning = False




        
#if __name__ == '__main__':
#    try:
#        rospy.init_node('auger_motor_calls', anonymous=True)
#        robot = AugerAPI()
#        print("Setting actuator position to 100")
#        robot.setActuatorPosition(position=100)
#        print("finished")
#    except rospy.ROSInterruptException:
#        pass