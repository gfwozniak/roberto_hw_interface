#!/usr/bin/env python

from robot_api import RobertoAPI
import threading
import time

class RobertoFunctions:

    def __init__(self):
        self.robot_api = RobertoAPI()
        self.e = threading.Event()

    # positional variables
        self.neutralbscrew = -10000
        self.neutralactuator = -5
        self.depositbscrew = -2000000
        self.depositactuator = 0
        self.minebscrew = -7000000
        self.mineactuator = -80
        self.raugerspeed = -.9
        self.faugerspeed = .9
        self.depositduration = 10

        self.bscrew_error = 50000
        self.actuator_error = 3

    def interrupt(self):
        self.e.set()
        print('call interrupt')
        while True:
            if(not self.e.is_set()):
                print("previous function ended")
                print(self.e.is_set())
                break
        print("sop motors")
        self.stopMotors()

    def delayinput(self):
        while True:
            if(not self.e.is_set()):
                break
        time.sleep(0.1)

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

#    def zeroBScrew(self):
#        self.interrupt()
#        self.robot_api.setBScrewPosition(8000000)
#        if(not self.waitUntilLimit()):
#            return
#        self.robot_api.setBScrewPosition(0)
#        if(not self.waitUntilEvent()):
#            return

# good
    def zeroAuger(self):
        self.interrupt()
        print("zero b screw")
        self.robot_api.setBScrewPosition(8000000)
        if(not self.waitUntilLimit()):
            print("zero auger exit!")
            return
        self.robot_api.setBScrewPosition(-10000)
        print("zero actuator")
        self.robot_api.setActuatorPosition(10000)
        if(not self.waitUntilTime(15)):
            print("zero auger exit!")
            return
        self.robot_api.zeroActuator()
        self.robot_api.setActuatorPosition(0)
        print("auger zeroed")
        if(not self.waitUntilEvent()):
            print("zero auger exit!")
            return

#    def returnToNeutral(self):
#        self.interrupt()
#        print("resetting bscrew")
#        bscrewposition = -10000
#        self.robot_api.setBScrewPosition(bscrewposition)
#        if(not self.waitUntilBScrewPosition(timeout=30,period=0.05,targetpos=bscrewposition,bscrew_error=6000)):
#            return
#        print("resetting actuator")
#        actuatorposition = -10
#        self.robot_api.setActuatorPosition(actuatorposition)
#        if(not self.waitUntilActuatorPosition(timeout=10,period=0.05,targetpos=actuatorposition,actuator_error=1)):
#            return
#        print("in neutral")
#        if(not self.waitUntilEvent()):
#            return
#    
#    def deposit(self):
#        self.interrupt()
#        print("depositing")
#        bscrewpos1 = -10000
#        self.robot_api.setBScrewPosition(bscrewpos1)
#        if(not self.waitUntilBScrewPosition(timeout=30,period=0.05,targetpos=bscrewpos1,bscrew_error=6000)):
#            return
#        print("set bscrew")
#        actuatorposition = 0
#        self.robot_api.setActuatorPosition(actuatorposition)
#        if(not self.waitUntilActuatorPosition(timeout=10,period=0.05,targetpos=actuatorposition,actuator_error=1)):
#            return
#        print("set actuator")
#        bscrewpos2 = -2000000
#        self.robot_api.setBScrewPosition(bscrewpos2)
#        if(not self.waitUntilBScrewPosition(timeout=30,period=0.05,targetpos=bscrewpos2,bscrew_error=6000)):
#            return
#        print("in depositin position")

    def moveBScrew(self, bscrewposition):
        print("before interrupt")
        self.interrupt()
        print("operation")
        self.robot_api.setBScrewPosition(bscrewposition)
        if(not self.waitUntilBScrewPosition(timeout=30,period=0.05,targetpos=bscrewposition,bscrew_error=100000)):
            return
        if(not self.waitUntilEvent()):
            print("final interrupt called")
            return
        print("what")

    def moveActuator(self, actuatorposition):
        self.interrupt()
        print("operation")
        self.robot_api.setActuatorPosition(actuatorposition)
        print("before awaits")
        if(not self.waitUntilActuatorPosition(timeout=30,period=0.05,targetpos=actuatorposition,actuator_error=3)):
            print("early exit")
            return
        print("awaiting next input")
        if(not self.waitUntilEvent()):
            print("standard exit")
            return

    def moveAuger(self, augervelocity, time):
        self.interrupt()
        self.robot_api.setAugerVelocity(augervelocity)
        if(not self.waitUntilTime(seconds=time)):
            self.robot_api.setAugerVelocity(0)
            return
        self.robot_api.setAugerVelocity(0)
        if(not self.waitUntilEvent()):
            return
    
    def neutralPosition(self):
        self.interrupt()
        # MOVE BSCREW TO NEUTRAL
        self.robot_api.setBScrewPosition(self.neutralbscrew)
        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.neutralbscrew,bscrew_error=self.bscrew_error)):
            return
        # MOVE ACTUATOR TO NEUTRAL
        self.robot_api.setActuatorPosition(self.neutralactuator)
        if(not self.waitUntilActuatorPosition(timeout=1000,period=0.05,targetpos=self.neutralactuator,actuator_error=self.actuator_error)):
            return
        # AWAIT
        if(not self.waitUntilEvent()):
            return
    
    def deposit(self):
        self.interrupt()
        # MOVE BSCREW TO NEUTRAL
        self.robot_api.setBScrewPosition(self.neutralbscrew)
        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.neutralbscrew,bscrew_error=self.bscrew_error)):
            return
        # MOVE ACTUATOR TO DEPOSIT
        self.robot_api.setActuatorPosition(self.depositactuator)
        if(not self.waitUntilActuatorPosition(timeout=1000,period=0.05,targetpos=self.depositactuator,actuator_error=self.actuator_error)):
            return
        # MOVE BSCREW TO DEPOSIT
        self.robot_api.setBScrewPosition(self.depositbscrew)
        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.depositbscrew,bscrew_error=self.bscrew_error)):
            return
        # REVERSE AUGER FOR 10 SECONDS
        self.robot_api.setAugerVelocity(self.raugerspeed)
        if(not self.waitUntilTime(seconds=self.depositduration)):
            self.robot_api.setAugerVelocity(0)
            return
        self.robot_api.setAugerVelocity(0)
        # MOVE BSCREW TO NEUTRAL
        self.robot_api.setBScrewPosition(self.neutralbscrew)
        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.neutralbscrew,bscrew_error=self.bscrew_error)):
            return
        # MOVE ACTUATOR TO NEUTRAL
        self.robot_api.setActuatorPosition(self.neutralactuator)
        if(not self.waitUntilActuatorPosition(timeout=1000,period=0.05,targetpos=self.neutralactuator,actuator_error=self.actuator_error)):
            return
        # AWAIT
        if(not self.waitUntilEvent()):
            return
    
    def mine(self):
        self.interrupt()
        # MOVE BSCREW TO NEUTRAL
        self.robot_api.setBScrewPosition(self.neutralbscrew)
        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.neutralbscrew,bscrew_error=self.bscrew_error)):
            return
        # MOVE ACTUATOR TO MINE
        self.robot_api.setActuatorPosition(self.mineactuator)
        if(not self.waitUntilActuatorPosition(timeout=1000,period=0.05,targetpos=self.mineactuator,actuator_error=self.actuator_error)):
            return
        # MOVE BSCREW TO MINE AND SPIN AUGER
        self.robot_api.setBScrewPosition(self.minebscrew)
        self.robot_api.setAugerVelocity(self.faugerspeed)
        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.minebscrew,bscrew_error=self.bscrew_error)):
            self.robot_api.setAugerVelocity(0)
            return
        self.robot_api.setAugerVelocity(0)
        # MOVE BSCREW TO NEUTRAL
        self.robot_api.setBScrewPosition(self.neutralbscrew)
        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.neutralbscrew,bscrew_error=self.bscrew_error)):
            return
        # MOVE ACTUATOR TO NEUTRAL
        self.robot_api.setActuatorPosition(self.neutralactuator)
        if(not self.waitUntilActuatorPosition(timeout=1000,period=0.05,targetpos=self.neutralactuator,actuator_error=self.actuator_error)):
            return
        # AWAIT
        if(not self.waitUntilEvent()):
            return

# Waiting events
    
    def waitUntilLimit(self):
        mustend = time.time() + 60
        while time.time() < mustend:
            if (self.robot_api.limit_switch_position): 
                return True
            if (self.e.is_set()):
                self.e.clear()
                print("interrupt called and cleared")
                return False
            time.sleep(.05)
        return True

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
        return True

    def waitUntilBScrewPosition(self, timeout, period, targetpos, bscrew_error):
        mintarget = targetpos - bscrew_error
        maxtarget = targetpos + bscrew_error
        mustend = time.time() + timeout
        while time.time() < mustend:
            if (self.robot_api.position[2] > mintarget and self.robot_api.position[2] < maxtarget): 
                print("target reached")
                return True
            if (self.e.is_set()):
                print("interrupt called")
                self.e.clear()
                return False
            time.sleep(period)
        print("timeout")
        return True
    
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