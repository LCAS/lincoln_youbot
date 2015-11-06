#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Teleop(object):
    
    def __init__(self, linearAxisIndex = 1, angularAxisIndex = 0):
        rospy.init_node('picksie_teleop')

        self._LinearAxisIndex = rospy.get_param("~linearAxisIndex", linearAxisIndex)
        self._AngularAxisIndex = rospy.get_param("~angularAxisIndex", angularAxisIndex)
        self._LinearScalingFactor = rospy.get_param("~linearScalingFactor", 0.5)
        self._AngularScalingFactor = rospy.get_param("~angularScalingFactor", 0.5)
        
        rospy.loginfo("Starting teleop node with linear axis %d and angular axis %d" % (self._LinearAxisIndex, self._AngularAxisIndex))
        # subscriptions
        rospy.Subscriber("joy", Joy, self._HandleJoystickMessage)
        self._VelocityCommandPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    
    def _HandleJoystickMessage(self, joyMessage):
        #rospy.logwarn("Handling joystick message: " + str(joyMessage))
        if joyMessage.buttons[2]:
            velocityCommand = Twist()
            velocityCommand.linear.x = self._LinearScalingFactor * joyMessage.axes[self._LinearAxisIndex]
            velocityCommand.angular.z = self._AngularScalingFactor * joyMessage.axes[self._AngularAxisIndex]    		
        else:
            velocityCommand = Twist()
            velocityCommand.linear.x = 0.0
            velocityCommand.angular.z = 0.0
        self._VelocityCommandPublisher.publish(velocityCommand)


if __name__ == '__main__':
    teleop = Teleop()
    rospy.spin()