#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'demo_basic'
tmux new-window -t $SESSION:1 -n 'RViz'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roslaunch picksie_simulation picksie_sim.launch"
tmux resize-pane -U 30

tmux select-pane -t 1
tmux send-keys "roslaunch youbot_moveit move_group.launch"
tmux split-window -v

tmux select-pane -t 2
tmux send-keys "rosrun moveit_commander moveit_commander_cmdline.py"
tmux select-pane -t 0

tmux select-window -t $SESSION:1
tmux send-keys "rosrun rviz rviz"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on