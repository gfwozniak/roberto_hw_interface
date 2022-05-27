#!/usr/bin/env python

from concurrent.futures import thread
from math import dist
from robot_api import RobertoAPI
import threading
import time

class RobertoFunctions:

    def __init__(self):
        self.robot_api = RobertoAPI()
        self.e = threading.Event()
        self.d = threading.Event()

    # positional variables
        self.neutralbscrew = -10000
        self.neutralactuator = -17
        self.depositbscrew = -3500000
        self.depositactuator = -2
        self.minebscrew = -7000000
        self.mineactuator = -80
        self.raugerspeed = -.9
        self.faugerspeed = .9
        self.depositduration = 20
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
        print("start delay")
        time.sleep(0.1)
        while True:
            if(not self.e.is_set()):
                break
        time.sleep(0.1)
        print("end delay")

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

# AUGER MOTOR COMMANDS
    def noMotorCommand(self):
        self.interrupt()
        print("motors stopped")
        if(not self.waitUntilEvent()):
            return

    def zeroAuger(self):
        self.interrupt()
        print("zero b screw")
        self.robot_api.setBScrewSpeed(2500000)
        self.robot_api.setBScrewPosition(position=8000000)
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

    def neutralPosition(self):
        self.interrupt()
        # MOVE BSCREW TO NEUTRAL
        self.robot_api.setBScrewSpeed(2500000)
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
        self.robot_api.setBScrewSpeed(2500000)
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
        self.robot_api.setBScrewSpeed(8000)
        self.robot_api.setBScrewPosition(self.minebscrew)
        self.robot_api.setAugerVelocity(self.faugerspeed)
        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.minebscrew,bscrew_error=self.bscrew_error)):
            self.robot_api.setAugerVelocity(0)
            return
        # MOVE BSCREW TO NEUTRAL
        self.robot_api.setBScrewSpeed(2500000)
        self.robot_api.setAugerVelocity(0)
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

# DRIVE FUNCTIONS

    def driveDistance(self):
        self.interrupt()
        self.d.set()
        self.robot_api.setDrivetrainVelocity(linearvelocity=5000,angularvelocity=0) # fix, won't move unless large value
        if(not self.waitUntilDistance(period=0.1,distance=self.robot_api.distance_to_sieve)):
            print("Forced exit")
            self.d.clear()
            return
        print("target reached")
        self.robot_api.setDrivetrainVelocity(linearvelocity=0,angularvelocity=0) 
        self.d.clear()
    
#    def depositDrive(self):
#        self.interrupt()
#        self.d.set()
#        self.robot_api.setDrivetrainVelocity(linearvelocity=100)
#        if(not self.waitUntilDistance(period=0.1,distance=self.robot_api.distance_to_sieve)):
#            print("Forced exit")
#            self.d.clear()
#            return
#        print("target reached")
#        self.d.clear()
#        self.robot_api.setBScrewSpeed(2500000)
#        self.robot_api.setBScrewPosition(self.neutralbscrew)
#        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.neutralbscrew,bscrew_error=self.bscrew_error)):
#            return
#        # MOVE ACTUATOR TO DEPOSIT
#        self.robot_api.setActuatorPosition(self.depositactuator)
#        if(not self.waitUntilActuatorPosition(timeout=1000,period=0.05,targetpos=self.depositactuator,actuator_error=self.actuator_error)):
#            return
#        # MOVE BSCREW TO DEPOSIT
#        self.robot_api.setBScrewPosition(self.depositbscrew)
#        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.depositbscrew,bscrew_error=self.bscrew_error)):
#            return
#        # REVERSE AUGER FOR 10 SECONDS
#        self.robot_api.setAugerVelocity(self.raugerspeed)
#        if(not self.waitUntilTime(seconds=self.depositduration)):
#            self.robot_api.setAugerVelocity(0)
#            return
#        self.robot_api.setAugerVelocity(0)
#        # MOVE BSCREW TO NEUTRAL
#        self.robot_api.setBScrewPosition(self.neutralbscrew)
#        if(not self.waitUntilBScrewPosition(timeout=1000,period=0.05,targetpos=self.neutralbscrew,bscrew_error=self.bscrew_error)):
#            return
#        # MOVE ACTUATOR TO NEUTRAL
#        self.robot_api.setActuatorPosition(self.neutralactuator)
#        if(not self.waitUntilActuatorPosition(timeout=1000,period=0.05,targetpos=self.neutralactuator,actuator_error=self.actuator_error)):
#            return
#        # AWAIT
#        if(not self.waitUntilEvent()):
#            return
        

# Waiting events
    
    def waitUntilLimit(self):
        print("BEFORE")
        print(self.e.is_set())
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

    def waitUntilDistance(self, period, distance):
#        if (distance < 0):
#            distance = -1 * distance
                                                                                                                    # check sign
        targetpos = self.robot_api.position[5] + distance
        while True:
            if (self.robot_api.position[5] > targetpos):
                print("position target reached")
                return True
            if (self.e.is_set()):
                print("interrupt called")
                self.e.clear()
                return False
            time.sleep(period)
