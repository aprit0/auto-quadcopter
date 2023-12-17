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
echo -e "alias drone'cd ~/ros2_ws/src/auto-quadcopter/auto-quadcopter/Drone'"
echo -e "alias base'cd ~/ros2_ws/src/auto-quadcopter/auto-quadcopter/Base'"
echo -e "\nalias build='cd ~/ros2_ws && colcon build && cd -'" >> ~/.bashrc

