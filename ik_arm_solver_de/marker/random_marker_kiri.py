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
####1000 marker random joint   
def makeBox(): 
    global solved,error,Done
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.id = aaa
    marker.scale.x = 0.035
    marker.scale.y = 0.035
    marker.scale.z = 0.035
    
    if (status == "IK_Solved"):
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        solved +=1
    elif (status == "IK_Error"):
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        error +=1
    else:
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
    
    marker.color.a = 1.0
    
    marker.pose.position.x = pos[0] 
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]

    marker.pose.orientation.x = ori[0]
    marker.pose.orientation.y = ori[1]
    marker.pose.orientation.z = ori[2]
    marker.pose.orientation.w = ori[3]
    print("position", pos)
    print("orientation", ori)
    print("status", status)
    print ("Berhasil", solved)
    print("Error", error)
    print("Marker",aaa)
    if (aaa >= 1000):
        Done = True
       # print("Done", Done)
   
    print("======================================================")
    return marker  

def mytopic_callback(msg):
    global status, aaa
    status = msg.hasil
    pos[0] = msg.position_x
    pos[1] = msg.position_y
    pos[2] = msg.position_z
    ori[0] = msg.orientation_x
    ori[1] = msg.orientation_y
    ori[2] = msg.orientation_z
    ori[3] = msg.orientation_w
    if (Done == False):
        marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=1)
        markerbasics_object = makeBox()
        marker_objectlisher.publish(markerbasics_object)
 
    aaa+=1
  

def run():
    rospy.Subscriber('chatter', jr, mytopic_callback)
   #rospy.Subscriber('pose_random', PoseStamped , mytopic_callback)
    rospy.spin()
   
rospy.init_node('marker_basic_node', anonymous=True)
rate = rospy.Rate(100) 
if __name__ == '__main__':
    try:
         # while not rospy.is_shutdown():
               run()
    
          #    rate.sleep()
    except rospy.ROSInterruptException:
        pass
