#!/usr/bin/env python

from auger_motor_api import AugerAPI

class AugerFunctions:

    def __init__(self):
        self.augerapi = AugerAPI()

    def returnToNeutral(self):
        self.augerapi.setBScrewPosition(position=-5000)
        self.augerapi.setActuatorPosition(position=-10)

    def deployAuger(self):
        if (self.augerapi.bscrew_position > 20000 or self.augerapi.bscrew_position < -20000):
            pass
        self.augerapi.setActuatorPosition(position=-86)
        self.augerapi.setBScrewPositionAndAugerVelocity(position=-7000000, velocity=1)
        self.returnToNeutral()

    def depositAuger(self):
        if (self.augerapi.bscrew_position > 20000 or self.augerapi.bscrew_position < -20000):
            pass
        self.augerapi.setActuatorPosition(position=-3)
        self.augerapi.setBScrewPosition(position=-2000000)
        self.augerapi.setAugerVelocityForDuration(velocity=-1, seconds=6)
        self.returnToNeutral()

    def zeroAuger(self):
        self.augerapi.zeroBScrew()
        self.returnToNeutral()




        
#if __name__ == '__main__':
#    try:
#        rospy.init_node('auger_motor_calls', anonymous=True)
#        robot = AugerAPI()
#        print("Setting actuator position to 100")
#        robot.setActuatorPosition(position=100)
#        print("finished")
#    except rospy.ROSInterruptException:
#        pass