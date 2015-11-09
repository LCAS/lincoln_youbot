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
        self.joint_names=['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']#, 'gripper_finger_joint_l', 'gripper_finger_joint_r']
        #self.def_pos=[3.14,0.058,-0.4834,2.4,2.88, 0.5, 0.5]
        
        self.def_pos=[3.0, 2.61, -0.91, 0.25, 2.88, 0.0115, 0.0]
#{'name': 'gripper_finger_joint_r', 'pos': 0.0115}
#        self.drop_pos= [5.84, 0.49, -1.0, 0.083, 2.88]
        self.drop_pos= [3.0, 0.011, -2.25, 0.25, 0.111, 0.0]
#{'name': 'gripper_finger_joint_r', 'pos': 0.0}

        self._xymode=False
        self.turbo = 1
        self.arm_disabled=True
        
        self._LinearAxisIndex = rospy.get_param("~linearAxisIndex", linearAxisIndex)
        self._AngularAxisIndex = rospy.get_param("~angularAxisIndex", angularAxisIndex)
        self._LinearScalingFactor = rospy.get_param("~linearScalingFactor", 0.5)
        self._AngularScalingFactor = rospy.get_param("~angularScalingFactor", 0.5)
        
        rospy.loginfo("Starting teleop node with linear axis %d and angular axis %d" % (self._LinearAxisIndex, self._AngularAxisIndex))
        

        for i in self.joint_names:
            joint={}
            joint['name']=i
            self._joints.append(joint)
            
        joint={}
        joint['name']= 'gripper_finger_joint_r'
        self._joints.append(joint)

        #print self._joints

        # subscriptions
        rospy.Subscriber("/joint_states", JointState, self._HandleJointStateMessage)
        rospy.Subscriber("/joy", Joy, self._HandleJoystickMessage)

        self._BricsCmdPublisher = rospy.Publisher("/arm_1/arm_controller/position_command", brics_actuator.msg.JointPositions, queue_size=1)
        self._GripperCmdPublisher = rospy.Publisher("/arm_1/gripper_controller/position_command", brics_actuator.msg.JointPositions, queue_size=1)
                
        self._VelocityCommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.sleep(2)
        self.go_to_search_pos()


    def go_to_search_pos(self):
        print "going to search pos"
        for i in range(0,len(self.joint_names)):
            arm_cmd = brics_actuator.msg.JointPositions()
            j_cmd= brics_actuator.msg.JointValue()
            j_cmd.joint_uri = self.joint_names[i]
            j_cmd.unit = 'rad'
            j_cmd.value = self.def_pos[i]
            arm_cmd.positions.append(j_cmd)
            print arm_cmd
            self._BricsCmdPublisher.publish(arm_cmd)
            rospy.sleep(1)
        self.arm_disabled=False
        print "Done"

    def go_to_drop_pos(self):
        print "going to drop pos"
        for i in range(0,len(self.joint_names)):
            arm_cmd = brics_actuator.msg.JointPositions()
            j_cmd= brics_actuator.msg.JointValue()
            j_cmd.joint_uri = self.joint_names[i]
            j_cmd.unit = 'rad'
            j_cmd.value = self.drop_pos[i]
            arm_cmd.positions.append(j_cmd)
            #print arm_cmd
            self._BricsCmdPublisher.publish(arm_cmd)
            rospy.sleep(1)
        self.arm_disabled=False


    def toggle_gripper(self):
        print self._joints[5]
        grp_cmd = brics_actuator.msg.JointPositions()
        j_cmd= brics_actuator.msg.JointValue()
        j_cmd.joint_uri = self._joints[5]['name']
        j_cmd.unit = 'm'
        if self._joints[5]['pos'] < 0.005:
            j_cmd.value = 0.0115
        else:
            j_cmd.value = 0.0
        grp_cmd.positions.append(j_cmd)        
        self._GripperCmdPublisher.publish(grp_cmd)


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
                velocityCommand.linear.y = self._LinearScalingFactor * joyMessage.axes[3]
                velocityCommand.angular.z = self._AngularScalingFactor * joyMessage.axes[self._AngularAxisIndex]    		
            else:
                velocityCommand = Twist()
                velocityCommand.linear.x = self._LinearScalingFactor * joyMessage.axes[self._LinearAxisIndex]
                velocityCommand.linear.y = self._LinearScalingFactor * joyMessage.axes[self._AngularAxisIndex]
                velocityCommand.angular.z = self._AngularScalingFactor * joyMessage.axes[3]
        else:
            velocityCommand = Twist()
            velocityCommand.linear.x = 0.0
            velocityCommand.linear.y = 0.0
            velocityCommand.angular.z = 0.0
        self._VelocityCommandPublisher.publish(velocityCommand)
        
        if joyMessage.buttons[3]:
            self.go_to_search_pos()
            self.arm_disabled=True
            
        if joyMessage.buttons[0]:
            self.go_to_drop_pos()
            self.arm_disabled=True


        if joyMessage.buttons[5]:
            print self._joints
            self.toggle_gripper()

        
        if not self.arm_disabled:
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
                    if j_cmd.value < 0.011 :
                       j_cmd.value = 0.011
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
                    if j_cmd.value < 0.011 :
                       j_cmd.value = 0.011
                arm_cmd.positions.append(j_cmd)
    
    
            if joyMessage.buttons[0]:
                j_cmd= brics_actuator.msg.JointValue()
                j_cmd.joint_uri = self._joints[4]['name']
                j_cmd.unit = 'rad'
                j_cmd.value = 2.90
                arm_cmd.positions.append(j_cmd)
    
    
#            if abs(joyMessage.axes[3])>0.5:
#                j_cmd= brics_actuator.msg.JointValue()
#                if joyMessage.axes[3] > 0:
#                    j_cmd.joint_uri = self._joints[4]['name']
#                    j_cmd.unit = 'rad'
#                    j_cmd.value = self._joints[4]['pos'] + 0.174533
#                    if j_cmd.value > 5.64 :
#                       j_cmd.value = 5.64
#                else:
#                    j_cmd.joint_uri = self._joints[4]['name']
#                    j_cmd.unit = 'rad'
#                    j_cmd.value = self._joints[4]['pos'] - 0.174533
#                    if j_cmd.value < 0.111 :
#                       j_cmd.value = 0.111
#                arm_cmd.positions.append(j_cmd)
    
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
                            
            

if __name__ == '__main__':
    teleop = Teleop()
    rospy.spin()