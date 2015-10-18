#!/bin/bash
echo "Starting Gazebo simulation!.."
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch youbot_gazebo_robot youbot.launch
echo "Done!"
