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
aaa = 1

solved = 0
error = 0
Done = False
abc = True
hehe = True
y = -0.2
z = 0.2
a = 0.04
b = 0.04
bbb = 1
####1000 marker random joint   
def makeBox(): 
    global solved,error,Done, bbb, aaa, hehe
    if (hehe):
        aaa = 1
    hehe = False
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.id = aaa
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    
    if (status == "IK_Solved"):
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        solved +=1
    #elif (status == "IK_Error"):
    else:
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        error +=1
    
    marker.color.a = 1.0
    
    marker.pose.position.x = pos[0] 
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]

    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
    print("position", pos)
    print("orientation", ori)
    # print("status", status)
    # print ("Berhasil", solved)
    # print("Error", error)
    # #print("Marker",bbb)
    #bbb+=1
 
    print("======================================================")
    return marker  

def makeBox2(pos_x,pos_y,pos_z): 
    marker2 = Marker()
    marker2.header.frame_id = "base_link"
    marker2.type = Marker.CUBE
    marker2.action = Marker.ADD
    marker2.id = aaa
    marker2.scale.x = 0.02
    marker2.scale.y = 0.02
    marker2.scale.z = 0.02
    #marker2.frame_locked = True 
    marker2.color.r = 0
    marker2.color.g = 0
    marker2.color.b = 1

    marker2.color.a = 1.0
    
    marker2.pose.position.x = pos_x 
    marker2.pose.position.y = pos_y
    marker2.pose.position.z = pos_z

    marker2.pose.orientation.x = 0
    marker2.pose.orientation.y = 0
    marker2.pose.orientation.z = 0
    marker2.pose.orientation.w = 1
    return marker2  


def mytopic_callback(msg):
    global status, aaa, bbb
    status = msg.hasil
    pos[0] = msg.position_x
    pos[1] = msg.position_y
    pos[2] = msg.position_z
    ori[0] = msg.orientation_x
    ori[1] = msg.orientation_y
    ori[2] = msg.orientation_z
    ori[3] = msg.orientation_w
    pubb= rospy.Publisher('/marker_basic', Marker, queue_size=1)
    marker1 = makeBox()
    pubb.publish(marker1)
    aaa+=1
  

def run():
    global aaa,bbb,abc, y,z,a,b
    rospy.init_node('marker_basic_node', anonymous=True)
    rate = rospy.Rate(1)
    rospy.Subscriber('chatter', jr, mytopic_callback)
   #rospy.Subscriber('pose_random', PoseStamped , mytopic_callback)
        
    for i in range (25):
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
        time.sleep(0.5)
        print("bbb",bbb)
        #
        pub = rospy.Publisher('/marker_basic', Marker, queue_size=1)
        marker2 = makeBox2(0.3,y,z)
        pub.publish(marker2)
        aaa+=1
        bbb+=1
        #rospy.loginfo(marker2)
    rospy.spin()
    #rate.sleep()


if __name__ == '__main__':
    try:
         while not rospy.is_shutdown():
               run()
    
               #rate.sleep()
    except rospy.ROSInterruptException:
        pass
