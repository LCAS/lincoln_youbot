#!/usr/bin/env python
from picksie import *

def Init():
    print "*******************************"
    print "RANDOM - Moves youbot to random target"
    print "*******************************"
    #Initialise the youbot object
    youbot = Youbot()
    #Move to random target
    youbot.Print_Joint_Values()
        #Finally - shutdown the robot
    youbot.Shutdown()
    

if __name__ == '__main__':
    try:
            Init()
    except rospy.ROSInterruptException:
        pass

