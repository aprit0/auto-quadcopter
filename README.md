# auto-quadcopter
Autonomous Quadcopter

## Background
# Main:
  Utilising a self developed flightcontroller, rpi controls the drone and interacts with the base over wifi.
  
# Arduino:
  A tested combination of rpi brain and arduino pwm/C based loop. Due to i2c redundancies, the arduino was shown to be not required. As I was using an arduino nano, the hardware speed is too low to provide a meaningful benefit. This method was tried second and dropped in favour of a complete pi based drone.
  
# Multiwii
  A combination of rpi brain, arduino nano receiving i2c commands and transferring them over PPM to an arduino running MultiWii. The benefit was supposed to be reduced development time with an out of the box open source solution. Due to numerous issues, compatability and expanding hardware requirements this method was tried first and dropped.

Project Overview:
Base Station
| Component | Hardware | Software |
| :-------- | :------- | :------- |
| Base Flight Controller | XBOX Controller \n Laptop | |
  Hardware:
  - XBOX One Controller (Wired)
  - Laptop running ROS2
  - Router
  Nodes:
  - joystick_node

Drone
  Required
  - Raspberry pi running ROS2
  - BNO085 9DOF IMU
  - MPL3115A2 Altimeter
  Features:
  -  
  Nodes:
  
  
  
  
