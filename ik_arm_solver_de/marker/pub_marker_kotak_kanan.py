#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from std_msgs.msg import Float64, String
from ik_arm_solver_de.msg import jr
from geometry_msgs.msg import PoseStamped
import time
pos = [0,0,0]
ori = [0,0,0,0]
status = "loading"

solved = 0
error = 0
Done = False
abc = True
x = 0.3
y = -0.2
z = 0.2
a = 0.04
b = 0.04
aaa = 0
def run():
    global aaa,bbb,abc, y,z,a,b, Done
       
    if(abc):
        y += a
        if (y >= -0.2 or y <= -0.36):
            a *= -1
            abc = False		
    else:
        z += b
        abc = True

    if (y <= -0.36):
        y = -0.36
    elif (y >= -0.2):
        y = -0.2
    if (z >= 0.36):
        z = 0.36
    elif (z <= 0.2):
        z = 0.2
    time.sleep(3)
    pub = rospy.Publisher('pose', PoseStamped, queue_size=1)
    msg = PoseStamped()
    msg.pose.position.x = x  
    msg.pose.position.y = y 
    msg.pose.position.z = z
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0
    msg.pose.orientation.w = 1
    aaa+=1
    if (aaa >= 23):
        Done = True
    if (Done):
        print("Done")
    else:
        pub.publish(msg)
    
        
        rospy.loginfo(msg)
    #rate.sleep()


if __name__ == '__main__':
    try:
         rospy.init_node('nodee', anonymous=True)
         rate = rospy.Rate(1)
    
         while not rospy.is_shutdown():
               run()
    
               #rate.sleep()
    except rospy.ROSInterruptException:
        pass
