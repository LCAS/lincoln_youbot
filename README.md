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
