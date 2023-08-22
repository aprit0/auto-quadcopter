#! /bin/bash
sudo apt update && sudo apt upgrade

# Setup environment
## VPN

## Random
sudo apt install i2c-tools
sudo apt install iwlist
## ROS
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
echo -e "\nalias foxglove='ros2 launch foxglove_bridge foxglove_bridge_launch.xml'" >> ~/.bashrc

