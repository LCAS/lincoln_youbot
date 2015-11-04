#!/usr/bin/env python
from simulation import *

def Init():
    print "*******************************"
    print "DEFAULT_POSITION - Moves the Youbot to the default position required for the vision tracking demo"
    print "*******************************"
    #Initialise the youbot object
    youbot = Youbot()
    #move the robot arm to a joint space target
    youbot.Target_JS([3.0662607633948866,
                      0.5601688277178765,
                      -0.901226487118465,
                      2.4373301489729045,
                      2.891868967060404])
    #Finally - shutdown the robot
    youbot.Shutdown()
if __name__ == '__main__':
    try:
            Init()
    except rospy.ROSInterruptException:
        pass
