#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'robot'
tmux new-window -t $SESSION:1 -n 'teleop'
tmux new-window -t $SESSION:2 -n 'whycon'


tmux select-window -t $SESSION:0
tmux send-keys "roslaunch youbot_driver_ros_interface youbot_driver.launch" 

tmux select-window -t $SESSION:1
tmux send-keys "roslaunch picksie_demo teleop.launch"

tmux select-window -t $SESSION:2
tmux send-keys "rosrun usb_cam usb_cam_node _framerate:=5"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
