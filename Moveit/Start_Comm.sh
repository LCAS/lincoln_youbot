#!/bin/bash
echo "Starting Moveit! Commander.."
cd ~/catkin_ws
source ./devel/setup.bash
rosrun moveit_commander moveit_commander_cmdline.py
echo "Done!"
