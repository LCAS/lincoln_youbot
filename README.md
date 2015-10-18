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
4)Start Moveit!
```
roslaunch youbot_moveit move_group.launch
```
5)Start Moveit! commander
```
rosrun moveit_commander moveit_commander_cmdline.py
```
6)The following code may be used to move the arm into the candle position
```
> use arm_1
arm_1> go candle
```
##Moveit with rospy
See sample code:
```
#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

def Init():
	pub = rospy.Publisher('chatter',String)
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('helloworld',anonymous=True)
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
	group = moveit_commander.MoveGroupCommander("arm_1")
	
	group_var_vals = group.get_current_joint_values()
	group_var_vals[0] = 0.1
	group_var_vals[3] = 0.9
	group.set_joint_value_target(group_var_vals)
	plan = group.plan()
	group.go()
	
	print group_var_vals

if __name__ == '__main__':
	try:
		Init()
	except rospy.ROSInterruptException:
		pass
			
```
#Running the demo program
This demo program will launch terminal windows for the gazebo simulation, and moveit/moveit commander.

1) Get files from project repo
```
sudo git clone https://github.com/LCAS/lincoln_youbot.git
```
2) Open and new terminal, and execute the demo bash file
```
bash lincoln_youbot/Demo/Demo.sh
```
