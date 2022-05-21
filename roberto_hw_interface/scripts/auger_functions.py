#!/usr/bin/env python

from robot_api import RobertoAPI

class AugerFunctions:

    def __init__(self):
        self.augerapi = RobertoAPI()
        self.isRunning = False

    def returnToNeutral(self):
        self.isRunning = True
        self.augerapi.setBScrewPosition(position=-5000)
        self.augerapi.setActuatorPositionTimeout(position=1010, seconds=10)
        self.isRunning = False

    def deployAuger(self):
        self.isRunning = True
        if (self.augerapi.bscrew_position > 20000 or self.augerapi.bscrew_position < -20000):
            pass
        self.augerapi.setActuatorPositionTimeout(position=940, seconds=15)
        self.augerapi.setBScrewPositionAndAugerVelocity(position=-7000000, velocity=0.7)
        self.augerapi.setBScrewPositionAndAugerVelocity(position=-6000000, velocity=0.8)
        self.augerapi.setBScrewPositionAndAugerVelocity(position=-7000000, velocity=0.4)
        self.augerapi.setBScrewPositionAndAugerVelocity(position=-6000000, velocity=0.3)
        self.augerapi.setBScrewPositionAndAugerVelocity(position=-7000000, velocity=0.2)
        self.augerapi.setBScrewPositionAndAugerVelocity(position=-6000000, velocity=0.8)
        self.augerapi.setBScrewPositionAndAugerVelocity(position=-7000000, velocity=0.3)
        self.augerapi.setAugerVelocityForDuration(velocity=.7, seconds=10)
        print("I am alive")
        self.returnToNeutral()
        self.isRunning = False

    def depositAuger(self):
        self.isRunning = True
        if (self.augerapi.bscrew_position > 20000 or self.augerapi.bscrew_position < -20000):
            pass
        self.augerapi.setActuatorPositionTimeout(position=1040, seconds=15)
        self.augerapi.setBScrewPosition(position=-2000000)
        self.augerapi.setAugerVelocityForDuration(velocity=-.8, seconds=6)
        self.augerapi.setAugerVelocityForDuration(velocity=0, seconds=1)
        self.returnToNeutral()
        self.isRunning = False

    def zeroAuger(self):
        self.isRunning = True
        self.augerapi.zeroBScrew()
        self.returnToNeutral()
        self.isRunning = False




        
#if __name__ == '__main__':
#    try:
#        rospy.init_node('auger_motor_calls', anonymous=True)
#        robot = AugerAPI()
#        print("Setting actuator position to 100")
#        robot.setActuatorPosition(position=100)
#        print("finished")
#    except rospy.ROSInterruptException:
#        pass