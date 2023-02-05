# Setup:
Option A: crontab -e
Option B:
`sudo vim /etc/init.d/autocopter`

`#!/bin/bash
### BEGIN INIT INFO
# Provides: MyService
# Required-Start:    $all
# Required-Stop: 
# Default-Start:     5 
# Default-Stop:      6 
# Short-Description: Your service description
### END INIT INFO

/home/pi/ros2_ws/src/auto-quadcopter/Boot/boot.sh`

sudo chmod +x /etc/init.d/autocopter

sudo update-rc.d autocopter defaults