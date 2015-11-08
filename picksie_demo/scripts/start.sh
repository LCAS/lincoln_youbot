#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'robot'
tmux new-window -t $SESSION:1 -n 'move_base'
tmux new-window -t $SESSION:2 -n 'joystick'
tmux new-window -t $SESSION:3 -n 'moveit!'
tmux new-window -t $SESSION:4 -n 'whycon'


tmux select-window -t $SESSION:0
tmux send-keys "roslaunch youbot_driver_ros_interface youbot_driver.launch" 

tmux select-window -t $SESSION:1
tmux send-keys "roslaunch youbot_navigation_local move_base_local.launch"

tmux select-window -t $SESSION:2
tmux send-keys "roslaunch picksie_demo teleop.launch"

tmux select-window -t $SESSION:3
tmux send-keys "roslaunch youbot_moveit move_group.launch"

tmux select-window -t $SESSION:4
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "rosrun usb_cam usb_cam_node"

tmux select-pane -t 1
tmux send-keys "rosrun whycon whycon _targets:=1 _outer_diameter:=0.049 _target_frame:=/camera_frame /camera/image_rect_color:=/usb_cam/image_color"
tmux select-pane -t 0


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
