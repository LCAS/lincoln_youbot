#!/usr/bin/env python
from picksie import *

def Init():
    print "*******************************"
    print "VISION TRACKING DEMO - Main vision tracking demo script"
    print "*******************************"
    #Initialise the youbot object
    youbot = Youbot()
    #Initialise the tracker object
    tracker = Tracker()
    #Move to the robot to its default position, to start tracking
    Default(youbot)
    
    #Keep looping
    while True:
	#Check if tracker has data
	while tracker.HasData():
		#Youbot target will be updated here
		# 
		#
		print "update youbot target"
		tracker.Display()
		
		

    
    #Finally - shutdown the robot
    youbot.Shutdown()

#Move the robot to the default position to start tracking
def Default(yb):
     #move the robot arm to a joint space target
    yb.Target_JS([3.0662607633948866,
                      0.5601688277178765,
                      -0.901226487118465,
                      2.4373301489729045,
                      2.891868967060404])
    

if __name__ == '__main__':
    try:
            Init()
    except rospy.ROSInterruptException:
        pass
