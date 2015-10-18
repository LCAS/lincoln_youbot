#!/bin/bash
echo "Starting demo.."
gnome-terminal -e "bash Gazebo/Start.sh"
sleep 5
gnome-terminal -e "bash Moveit/Start.sh"
sleep 5
gnome-terminal -e "bash Moveit/Start_Comm.sh"
echo "Finished."
