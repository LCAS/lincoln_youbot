#!/bin/bash
echo "Starting Moveit!.."
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch youbot_moveit move_group.launch
echo "Done!"
