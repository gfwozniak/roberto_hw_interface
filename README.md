# DISCLAIMER: This software is deprecated and not maintained
This repository holds the ROS Melodic Package for College of DuPage's 2022 Lunabotics robot.

## What this code can be used for
- Running the 2022 Lunabotics robot on ROS melodic 
- Looking at how to compile and run the Phoenix Motor Shared Libraries for new roscpp applications, [A Guide on Making your own VEX Motor Integration in ROS](https://docs.google.com/document/d/1sTFA5TZgk0GMn26wAikqEjBo5B9KgYuq0BL8JprOFXA/edit?usp=sharing)

## Issues with this code
- This code is NOT a good example of ros_control (I don't think ros_control should be used moving forward)
- This code is NOT a good example of python scripting for inputs (I did it in a very hackish way due to time constraints)
- This code runs on ROS Melodic which has EOL May 2023

### To run the program on the Lunabotics robot
1. Install this package to the catkin_ws on both your remote computer and your workstation computer and run ```catkin_make``` to build
2. Make sure both devices are network configured
3. On the remote computer, run the following commands:
```./canableStart # while in the directory where this script is located ```
```roslaunch roberto_hw_interface jetson_setup.launch```
4. Meanwhile on the workstation computer, make sure an xbox controller is plugged in and run the following commmand:
```roslaunch roberto_hw_interface app_test.launch```
5. Follow the instructions in the terminal for inputs (it awaits one of the button presses to zero the motors before accepting any other inputs)
6. Controls:
Triggers and Left Joystick - Driving
X - Mine
Y - Deposit
A - Stop all motors
B - Neutral position
The other buttons are not standardized in labelling so I don't know what to call them but there are some other binds.

Special thanks to the rest of the 2022 programming team: Oliver Burrus, Nayal Merchant, Giovanni Zavalza, and Stavros Dellis
