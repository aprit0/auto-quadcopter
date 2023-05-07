#!/bin/sh
sudo tailscale up
python3 /home/pi/ros2_ws/src/auto-quadcopter/Drone/status_node.py
