# lincoln_youbot

##Testing the simulation environment

1)Launch the gazebo simulation:
``` roslaunch youbot_gazebo_robot youbot.launch  ```

2)Open a seperate terminal, and use the following code for testing the simulation:
```
rostopic pub  /arm_1/arm_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ["arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"]
points:
- positions: [0.6,0.0,0.0,0.0,0.0]
  velocities: [0.1,0.1,0.1,0.1,0.1]
  accelerations: [0.1,0.1,0.1,0.1,0.1]
  effort: [0]
  time_from_start: {secs: 1, nsecs: 0}"
```
---

##Setting up Moveit
1)Install essential ROS and Moveit! packages:
```
sudo apt-get install python-wstool ros-indigo-roscpp ros-indigo-pluginlib ros-indigo-urdf \
  ros-indigo-tf-conversions ros-indigo-joint-state-publisher \
  ros-indigo-robot-state-publisher ros-indigo-xacro \
  ros-indigo-moveit-core ros-indigo-moveit-ros-move-group \
  ros-indigo-moveit-planners-ompl ros-indigo-moveit-ros-visualization
```
2)Add the youbot_manipulation repository
```
cd ~/catkin_ws/src
git clone https://github.com/svenschneider/youbot-manipulation.git
cd ..
catkin_make
source ./devel/setup.bash
```
3)Launch simulation
```
roslaunch youbot_gazebo_robot youbot.launch 
```
4)Start Moveit! commander
```
rosrun moveit_commander moveit_commander_cmdline.py
```
5)The following code may be used to move the arm into the candle position
```
> use arm_1
arm_1> go candle
```
