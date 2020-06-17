#!/usr/bin/env bash
echo 1 > /proc/sys/vm/drop_caches
ip="$(ifconfig | grep -A 1 'eth0' | tail -1 | cut -d ':' -f 2 | cut -d ' ' -f 1)"
export ROS_IP=$ip
export ROS_IP
roslaunch ros_odrive position.launch