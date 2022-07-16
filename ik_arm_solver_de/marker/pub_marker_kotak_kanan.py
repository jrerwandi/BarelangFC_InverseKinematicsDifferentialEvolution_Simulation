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
move_y = True
x = 0.3
y = -0.2
z = 0.2
point_y = 0.04
point_z = 0.04
point = 0
def run():
    global point,move_y, y,z,point_y,point_z, Done
       
    if(move_y):
        y += point_y
        if (y >= -0.2 or y <= -0.36):
            point_y *= -1
            move_y= False		
    else:
        z += point_z
        move_y = True

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
    point+=1
    if (point >= 23):
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
