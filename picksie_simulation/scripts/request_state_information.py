#!/usr/bin/env python
from simulation import *

def Init():
    print "*******************************"
    print "REQUEST_STATE_INFORMATION - Prints youbot state information to the console"
    print "*******************************"
    #Initialise the youbot object
    youbot = Youbot()
    #Now print state information
    # -- Print robot state
    youbot.Print_State()
    # -- Print joint state values
    youbot.Print_Joint_Values()
    #Finally - shutdown the robot
    youbot.Shutdown()
if __name__ == '__main__':
    try:
            Init()
    except rospy.ROSInterruptException:
        pass
