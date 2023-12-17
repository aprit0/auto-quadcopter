# Autonomous Quadcopter
  An autonomous drone designed for semi autonomous flight. As I am no flying wiz, skill is supplemented with software! 
## STATUS
- [x] Build prototype hardware
- [x] Code and integrate flight controller sensors
- [x] Code and integrate base station
- [x] Roll/Pitch/Yaw PID Tuning
- [x] FIRST FLIGHT  
- [~] Code and integrate XYZ Hover - Awaiting Optical Flow Sensor  
- [ ] STABLE PLATFORM FLIGHT
- [x] Code and integrate GPS
- [ ] ROS RANGE TEST FLIGHT

## Small Scale Task Tracking
- [ ] Validate sensors in RVIZ 
-   [ ] Ensure OF twist is in the correct orientation and is un 

## Bugs
- [ ] BMS Current Sensor 


# Background
## Main
  Utilising a self developed flight controller, rpi controls the drone and interacts with the base over ROS.  
  
## Multiwii
  A combination of rpi brain, arduino nano receiving i2c commands and transferring them over PPM to an arduino running MultiWii. The benefit was supposed to be reduced development time with an out of the box open source solution. Due to numerous issues, compatability and expanding hardware requirements this method was tried first and dropped.  

## Arduino
  A tested combination of rpi brain and arduino pwm/C based loop. Due to i2c redundancies, the arduino was shown to be not required. As I was using an arduino nano, the hardware speed is too low to provide a meaningful benefit. This method was tried second and dropped in favour of a complete pi based drone.  
  
  

# Project Overview:
Using ROS2 nodes, the code base can be easily modular, adaptable and upgradeable. The simple wifi streaming of data allows for high quality communications at the cost of range.

## Base Station
| Component | Hardware | Software |
| :-------- | :------- | :------- |
| Base Station | XBOX Controller, Laptop, Router | joystick_node |

## Drone
| Component | Hardware | Software |
| :-------- | :------- | :------- |
| Base Flight Controller | RPi, BNO085 9DOF IMU | control_node, devices_node |
| XYZ Hover Control | USB2.0 Webcam, VL53L0X, MPL3115A2 Altimeter, (IMU) | TBD |
| GPS | BN880Q - Future RTX GPS? | devices_node |
| Camera Feed | Analogue Camera, TX, RX, Android Phone | N/A |
| BMS | Self Designed PCB | TBD: I2C |

## Features
- XYZ Stabilisation with optical flow
- Benefits of ROS ecosystem

## Conventions
- Orientation/Units: [REP103](https://www.ros.org/reps/rep-0103.html)



# Installation
## Dependencies
TOF: https://github.com/Gadgetoid/VL53L0X-python/blob/master/python/VL53L0X.py
..

## Boot
crontab -e, "@reboot ~/ros2_ws/src/auto-quadcopter/boot.sh"

# Learnings
## Optical Flow
This algorithm was coded and implemented. Loop speed was reasonable at 60Hz and able to accurately track magnitude and directions with both sparse and dense flow algorithms.   
Limitation: The hardware used was a webcam, this was limited to 30Hz BUT *Auto focus could cause every second image to be blurred*  
Fix: Buy Optical Flow hardware for lateral velocity

# Ros
## Handy lines
```
ros2 topic pub -r 10 base/ARM std_msgs/msg/Bool "{data: True}" 
```



