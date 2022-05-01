#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
#import os #from initialize controller script
from auger_functions import AugerFunctions
#from initializeController import initializeController
#import runnertest.py need to be in same dir?
# Author: Andrew Dai, gotten from the https://andrewdai.co/xbox-controller-ros.html#rosjoy
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
    # then converts the joysick inputs into Twist commands
    # axis 1 aka left stick vertical controls linear speed
    # axis 0 aka left stick horizonal controls angular speed
class JoystickPublisher:
    def __init__(self, Publisher, AugerFunctions):
        self.pub = Publisher
        self.robot = AugerFunctions

    def callback(self, message):
        twist = Twist()
        twist.linear.x = 4*message.axes[1]
        twist.angular.z = 4*message.axes[0]
        self.pub.publish(twist)

        #now lets see what happens when i click a button:
        #if x(0) is clicked: do something
        #if button is clicked:
        if (not self.robot.isRunning):
            if (message.buttons[0]):
                print("X(Playstation)        or A(Xbox) is pressed")
                self.robot.returnToNeutral()
            if (message.buttons[1]):
                print("Circle(Playstation    or B(Xbox) is pressed")
                self.robot.zeroAuger()
            if (message.buttons[2]):
                print("Square(Playstation)   or X(Xbox) is pressed")
                self.robot.deployAuger()
            if (message.buttons[3]):
                print("Triangle(Playstation) or Y(Xbox) is pressed")
                self.robot.depositAuger()

# Intializes everything
def start():
    rospy.init_node('Joy2RobotControl')
    robot = AugerFunctions()
    pub = rospy.Publisher('drivetrain_cmd', Twist, queue_size=5)
    joystick = JoystickPublisher(pub, robot)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, joystick.callback)
    # starts the node
    rospy.spin()

    
if __name__ == '__main__':
        #start the initialize controller script
#        initializeController()
        start()
