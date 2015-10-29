#!/usr/bin/env python
from simulation import *

def Init():
    print "*******************************"
    print "RANDOM - Moves youbot to random target"
    print "*******************************"
    #Initialise the youbot object
    youbot = Youbot()
    #Move to random target
    youbot.Random()
    #Finally - shutdown the robot
    youbot.Shutdown()
    

if __name__ == '__main__':
    try:
            Init()
    except rospy.ROSInterruptException:
        pass

