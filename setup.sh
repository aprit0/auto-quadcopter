#! /bin/bash
sudo apt update && sudo apt upgrade

# Setup environment
## VPN
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

## Random
sudo apt install i2c-tools
sudo apt install iwlist
## ROS
