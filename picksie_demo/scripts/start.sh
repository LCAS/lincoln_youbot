#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'simulation'
tmux new-window -t $SESSION:1 -n 'moveit!'
tmux new-window -t $SESSION:2 -n 'keyboard'
tmux new-window -t $SESSION:3 -n 'whycon'
tmux new-window -t $SESSION:4 -n 'RViz'

tmux select-window -t $SESSION:0
tmux send-keys "roslaunch picksie_simulation picksie_sim.launch" 

tmux select-window -t $SESSION:1
tmux send-keys "roslaunch youbot_moveit move_group.launch"

tmux select-window -t $SESSION:2
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "rosrun moveit_commander moveit_commander_cmdline.py"

tmux select-pane -t 1
tmux send-keys "rosrun teleop_twist_keyboard teleop_twist_keyboard.py"


tmux select-window -t $SESSION:3
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "ROS_NAMESPACE=usb_cam rosrun image_proc image_proc"

tmux select-pane -t 1
tmux send-keys "rosrun whycon whycon _targets:=1 _outer_diameter:=0.049 _target_frame:=/camera_frame /camera/image_rect_color:=/usb_cam/image_color"
tmux select-pane -t 0


tmux select-window -t $SESSION:4
tmux send-keys "rosrun rviz rviz"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
