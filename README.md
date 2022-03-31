# roberto_hw_interface
ros_control hardware interface for Roberto the Moon Roomba

## hardware requirements:
- You need a CANable usb to can device. Idk why but the phoenix libraries do not work on other can devices

## software requirements:
- You need ROS Melodic for this package to work

### after installing this package:
Go to the top directory of your catkin workspace where the source code of the ROS packages you'd like to use are. Then run:
```rosdep install --from-paths src --ignore-src -r -y``` then ```catkin_make``` to build the C++ nodes

### then after every system restart
Go find the canableStart.sh bash script and run it with the CANable device plugged in (should switch from blue to green light)

then to run the launch file:
```roslaunch roberto_hw_interface hw_interface.launch```
