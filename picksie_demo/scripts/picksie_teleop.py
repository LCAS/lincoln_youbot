#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

import brics_actuator.msg

#/JointPositions

class Teleop(object):
    
    def __init__(self, linearAxisIndex = 1, angularAxisIndex = 0):
        rospy.init_node('picksie_teleop')
        self._joints=[]
        joints=['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'gripper_finger_joint_l', 'gripper_finger_joint_r']
        self._xymode=False
        self.turbo = 1
        
        self._LinearAxisIndex = rospy.get_param("~linearAxisIndex", linearAxisIndex)
        self._AngularAxisIndex = rospy.get_param("~angularAxisIndex", angularAxisIndex)
        self._LinearScalingFactor = rospy.get_param("~linearScalingFactor", 0.5)
        self._AngularScalingFactor = rospy.get_param("~angularScalingFactor", 0.5)
        
        rospy.loginfo("Starting teleop node with linear axis %d and angular axis %d" % (self._LinearAxisIndex, self._AngularAxisIndex))
        

        for i in joints:
            joint={}
            joint['name']=i
            self._joints.append(joint)

        #print self._joints

        # subscriptions
        rospy.Subscriber("/joint_states", JointState, self._HandleJointStateMessage)
        rospy.Subscriber("/joy", Joy, self._HandleJoystickMessage)

        self._BricsCmdPublisher = rospy.Publisher("/arm_1/arm_controller/position_command", brics_actuator.msg.JointPositions, queue_size=1)
        self._VelocityCommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


    def _HandleJointStateMessage(self, jointMessage):
        for i in self._joints:
            if i['name'] in jointMessage.name:
                i['pos'] = jointMessage.position[jointMessage.name.index(i['name'])]
        #print self._joints
    
    def _HandleJoystickMessage(self, joyMessage):
        turbo = self.turbo
        if joyMessage.buttons[9]:
            self._xymode = not self._xymode

        if joyMessage.buttons[6]:
            self.turbo = 1.0
            print "Turbo = %d" %turbo
        if joyMessage.buttons[7]:
            self.turbo += 1.0
            print "Turbo = %d" %turbo

        if joyMessage.buttons[4]:
            if not self._xymode:
                velocityCommand = Twist()
                velocityCommand.linear.x = self._LinearScalingFactor * joyMessage.axes[self._LinearAxisIndex]
                #velocityCommand.linear.y = self._LinearScalingFactor * joyMessage.axes[3]
                velocityCommand.angular.z = self._AngularScalingFactor * joyMessage.axes[self._AngularAxisIndex]    		
            else:
                velocityCommand = Twist()
                velocityCommand.linear.x = self._LinearScalingFactor * joyMessage.axes[self._LinearAxisIndex]
                velocityCommand.linear.y = self._LinearScalingFactor * joyMessage.axes[self._AngularAxisIndex]
                #velocityCommand.angular.z = self._AngularScalingFactor * joyMessage.axes[3]
        else:
            velocityCommand = Twist()
            velocityCommand.linear.x = 0.0
            velocityCommand.linear.y = 0.0
            velocityCommand.angular.z = 0.0
        self._VelocityCommandPublisher.publish(velocityCommand)
        
        
        arm_cmd = brics_actuator.msg.JointPositions()
        
        if abs(joyMessage.axes[6])>0.5:
            j_cmd= brics_actuator.msg.JointValue()
            if joyMessage.axes[6] > 0:
                j_cmd.joint_uri = self._joints[0]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[0]['pos'] + (0.0174533*turbo)
                if j_cmd.value > 5.84 :
                   j_cmd.value = 5.84
            else:
                j_cmd.joint_uri = self._joints[0]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[0]['pos'] - (0.0174533*turbo)
                if j_cmd.value < 0.01 :
                   j_cmd.value = 0.01
            arm_cmd.positions.append(j_cmd)

        if abs(joyMessage.axes[7])>0.5:
            j_cmd= brics_actuator.msg.JointValue()
            if joyMessage.axes[7] > 0:
                j_cmd.joint_uri = self._joints[1]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[1]['pos'] + (0.0174533*turbo)
                if j_cmd.value > 2.617 :
                   j_cmd.value = 2.617 
            else:
                j_cmd.joint_uri = self._joints[1]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[1]['pos'] - (0.0174533*turbo)
                if j_cmd.value < 0.01 :
                   j_cmd.value = 0.01
            arm_cmd.positions.append(j_cmd)


        if joyMessage.buttons[0]:
            j_cmd= brics_actuator.msg.JointValue()
            j_cmd.joint_uri = self._joints[4]['name']
            j_cmd.unit = 'rad'
            j_cmd.value = 2.90
            arm_cmd.positions.append(j_cmd)



        if abs(joyMessage.axes[3])>0.5:
            j_cmd= brics_actuator.msg.JointValue()
            if joyMessage.axes[3] > 0:
                j_cmd.joint_uri = self._joints[4]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[4]['pos'] + 0.174533
                if j_cmd.value > 5.64 :
                   j_cmd.value = 5.64
            else:
                j_cmd.joint_uri = self._joints[4]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[4]['pos'] - 0.174533
                if j_cmd.value < 0.111 :
                   j_cmd.value = 0.111
            arm_cmd.positions.append(j_cmd)

        if abs(joyMessage.axes[4])>0.5:
            j_cmd= brics_actuator.msg.JointValue()
            if joyMessage.axes[4] > 0:
                j_cmd.joint_uri = self._joints[3]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[3]['pos'] + (0.0174533 * abs(joyMessage.axes[4]))
                if j_cmd.value > 3.42 :
                   j_cmd.value = 3.42 
            else:
                j_cmd.joint_uri = self._joints[3]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = self._joints[3]['pos'] - (0.0174533 * abs(joyMessage.axes[4]))
                if j_cmd.value < 0.0222 :
                   j_cmd.value = 0.0222
            arm_cmd.positions.append(j_cmd)


        if joyMessage.axes[5] < 0.0:
            j_cmd= brics_actuator.msg.JointValue()
            j_cmd.joint_uri = self._joints[2]['name']
            j_cmd.unit = 'rad'
            j_cmd.value = self._joints[2]['pos'] + 0.0174533
            if j_cmd.value > -0.0158 :
               j_cmd.value = -0.0158
            arm_cmd.positions.append(j_cmd)

        if joyMessage.axes[2] < 0.0:
            j_cmd= brics_actuator.msg.JointValue()
            j_cmd.joint_uri = self._joints[2]['name']
            j_cmd.unit = 'rad'
            j_cmd.value = self._joints[2]['pos'] - 0.0174533
            if j_cmd.value < -5.0655 :
               j_cmd.value = -5.0655 
            arm_cmd.positions.append(j_cmd)


        if len(arm_cmd.positions) >0:
            self._BricsCmdPublisher.publish(arm_cmd)            
                            
        if joyMessage.buttons[5]:
            print self._joints
            

if __name__ == '__main__':
    teleop = Teleop()
    rospy.spin()