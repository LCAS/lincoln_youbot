#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

#***********************
# YOUBOT CLASS
#***********************
class Youbot:
    #Class initialise method
    def __init__(self):
        print "Initialising youbot"
        #Initialise moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        #Initialise rospy
        rospy.init_node('simulation',anonymous=True)
        #Initialise robot commander object
        self.robot = moveit_commander.RobotCommander()
        #Initialise scene object
        self.scene = moveit_commander.PlanningSceneInterface()
        #Initialise move group object
        self.group = moveit_commander.MoveGroupCommander("arm_1")
        #Initialise RVIZ
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
        print "Initialisation finished."
        #Finally - print the initial robot state
        self.Print_State()
    #Move robot using currently planned target
    def Move(self):
        #move to target
        self.group.go()
    #Move robot to a random target
    def Random(self):
        print "Moving to random target"
        #generate the random target
        self.group.set_random_target()
        #move the robot to the target
        self.Move()
    def Target(self,x,y,z):
        print "Moving to cartesian target"
        #Set the target position
        self.group.set_position_target([x,y,z])
        #move to target
        self.Move()    
    #Print robot state
    def Print_State(self):
        print "PRINTING CURRENT ROBOT STATE"
        print self.robot.get_current_state()
        print " "
    #Robot shutdown
    def Shutdown(self):
        print "SHUTTING DOWN YOUBOT"
        moveit_commander.roscpp_shutdown()
	
