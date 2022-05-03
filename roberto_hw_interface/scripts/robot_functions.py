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
            if(not self.e.is_set()):
                break
        self.stopMotors()

    def delayinput(self):
        while True:
            if(not self.e.is_set()):
                break

    def initialMotors(self):
        if(not self.waitUntilJointStatePublish()):
            return
        self.stopMotors()
        if(not self.waitUntilEvent()):
            return

    def stopMotors(self):
        self.robot_api.setAugerVelocity(0.0)
        self.robot_api.setDrivetrainVelocity(0.0, 0.0)
        self.robot_api.setBScrewPosition(self.robot_api.position[2])
        self.robot_api.setActuatorPosition(self.robot_api.position[0])

    def noMotorCommand(self):
        self.interrupt()
        print("motors stopped")
        if(not self.waitUntilEvent()):
            return

    def zeroBScrew(self):
        self.interrupt()
        self.robot_api.setBScrewPosition(8000000)
        if(not self.waitUntilLimit()):
            return
        self.robot_api.setBScrewPosition(0)
        if(not self.waitUntilEvent()):
            return

    def zeroAuger(self):
        self.interrupt()
        print("zero b screw")
        self.robot_api.setBScrewPosition(8000000)
        if(not self.waitUntilLimit()):
            return
        self.robot_api.setBScrewPosition(-10000)
        print("zero actuator")
        self.robot_api.setActuatorPosition(10000)
        if(not self.waitUntilTime(5)):
            return
        self.robot_api.zeroActuator()
        self.robot_api.setActuatorPosition(0)
        print("auger zeroed")
        if(not self.waitUntilEvent()):
            return

    def returnToNeutral(self):
        self.interrupt()
        print("resetting bscrew")
        bscrewposition = -10000
        self.robot_api.setBScrewPosition(bscrewposition)
        if(not self.waitUntilBScrewPosition(timeout=30,period=0.05,targetpos=bscrewposition,bscrew_error=6000)):
            return
        print("resetting actuator")
        actuatorposition = -10
        self.robot_api.setActuatorPosition(actuatorposition)
        if(not self.waitUntilActuatorPosition(timeout=10,period=0.05,targetpos=actuatorposition,actuator_error=1)):
            return
        print("in neutral")
        if(not self.waitUntilEvent()):
            return
    
    def deposit(self):
        self.interrupt()
        print("depositing")
        bscrewpos1 = -10000
        self.robot_api.setBScrewPosition(bscrewpos1)
        if(not self.waitUntilBScrewPosition(timeout=30,period=0.05,targetpos=bscrewpos1,bscrew_error=6000)):
            return
        print("set bscrew")
        actuatorposition = 0
        self.robot_api.setActuatorPosition(actuatorposition)
        if(not self.waitUntilActuatorPosition(timeout=10,period=0.05,targetpos=actuatorposition,actuator_error=1)):
            return
        print("set actuator")
        bscrewpos2 = -2000000
        self.robot_api.setBScrewPosition(bscrewpos2)
        if(not self.waitUntilBScrewPosition(timeout=30,period=0.05,targetpos=bscrewpos2,bscrew_error=6000)):
            return
        print("in depositin position")

# Waiting events
    
    def waitUntilLimit(self):
        mustend = time.time() + 60
        while time.time() < mustend:
            if (self.robot_api.limit_switch_position): 
                return True
            if (self.e.is_set()):
                self.e.clear()
                return False
            time.sleep(.05)
        return False

    def waitUntilJointStatePublish(self):
        mustend = time.time() + 60
        while time.time() < mustend:
            if (self.robot_api.is_joint_initialized): 
                return True
            if (self.e.is_set()):
                self.e.clear()
                return False
            time.sleep(.05)
        return False

    def waitUntilEvent(self):
        while True:
            if (self.e.is_set()):
                self.e.clear()
                return False
            time.sleep(.05)

    def waitUntilTime(self, seconds):
        mustend = time.time() + seconds
        while time.time() < mustend:
            if (self.e.is_set()):
                self.e.clear()
                return False
            time.sleep(.05)
        return True
        
    def waitUntilActuatorPosition(self, timeout, period, targetpos, actuator_error):
        mintarget = targetpos - actuator_error
        maxtarget = targetpos + actuator_error
        mustend = time.time() + timeout
        while time.time() < mustend:
            if (self.robot_api.position[0] > mintarget and self.robot_api.position[0] < maxtarget): 
                return True
            if (self.e.is_set()):
                self.e.clear()
                return False
            time.sleep(period)
        return False

    def waitUntilBScrewPosition(self, timeout, period, targetpos, bscrew_error):
        mintarget = targetpos - bscrew_error
        maxtarget = targetpos + bscrew_error
        print(mintarget)
        print(maxtarget)
        print(self.robot_api.position[2])
        mustend = time.time() + timeout
        while time.time() < mustend:
            if (self.robot_api.position[2] > mintarget and self.robot_api.position[2] < maxtarget): 
                return True
            if (self.e.is_set()):
                self.e.clear()
                return False
            time.sleep(period)
        return False
    
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