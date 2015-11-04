#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String



#***********************
# TRACKER CLASS
#***********************
class Tracker:
    
    #Tracker call back method
    def GetData(self,data):
	self.data = data
    #initialise method
    def __init__(self):
        print "STARTING TRACKER"
	self.data = None
       	rospy.Subscriber("/whycon/poses",geometry_msgs.msg.PoseArray,self.GetData)
	print "TRACKER STARTED"
    #check if the tracker is getting data
    def HasData(self):
	#check data
	if self.data == None:
		return False
	elif self.data == []:
		return False
	else:
		
		return True

    #Get tracker position (x,y,z)
    def GetPosition(self):
	if self.HasData(): 
		return self.data.poses[0].position
	else:
		return 0
	
    #Get tracker orientation (x,y,z,w)
    def GetOrientation(self):
	if self.HasData():
		return self.data.poses[0].orientation
	else:
		return 0

    #print tracker position
    def PrintPosition(self):
	print "TRACKER POSITION"
	print self.GetPosition()
    #print tracker orientation
    def PrintOrientation(self):
	print "TRACKER ORIENTATION"
	print self.GetOrientation()
    #print current tracker data
    def Display(self):
        self.PrintPosition()
	self.PrintOrientation()
  
    
 

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
        #youbot has now been initialised
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
    def Target_Pose(self,x,y,z,qx,qy,qz,qw):
        print "Moving to cartesian target"
        #print current youbot pose
        print self.group.get_current_pose()
        #clear the previous target, if it exists
        self.group.clear_pose_targets()
        self.group.set_pose_target([x,y,z,qx,qy,qz,qw])
        self.Move()
	
    def Target_JS(self,j):
        print "Moving to joint space goal"
        #Get current joint values
        group_variable_values_new = self.group.get_current_joint_values()
        #print current values
        self.Print_Joint_Values()
        #now set the values
        group_variable_values_new[0] = j[0] #joint01
        group_variable_values_new[1] = j[1] #joint02
        group_variable_values_new[2] = j[2] #joint03
        group_variable_values_new[3] = j[3] #joint04
        group_variable_values_new[4] = j[4] #joint05
	
	#set the joint value target
        self.group.set_joint_value_target(group_variable_values_new)
        #plan route to target
        plan2 = self.group.plan()
        #pause while target is planned
        rospy.sleep(5)
        #now move to the target
        self.Move()
	

    #Print robot state
    def Print_State(self):
        print "CURRENT ROBOT STATE"
        print self.robot.get_current_state()
        print " "
    #Print joint state values
    def Print_Joint_Values(self):
        print "CURRENT JOINT SPACE VALUES"
        print self.group.get_current_joint_values()
        print " "
    
    #Robot shutdown
    def Shutdown(self):
        print "SHUTTING DOWN YOUBOT"
        moveit_commander.roscpp_shutdown()
	
