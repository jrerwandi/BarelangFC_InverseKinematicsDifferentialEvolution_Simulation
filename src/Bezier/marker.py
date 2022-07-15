#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from std_msgs.msg import Float64, String
#from ik_arm_solver_de.msg import jr
from geometry_msgs.msg import PoseStamped

pos = [0,0,0]
ori = [0,0,0,0]
status = "loading"
aaa = 0
####1000 marker random joint   
def makeBox(): 
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
    elif (status == "IK_Error"):
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
    else:
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
    
    marker.color.a = 1.0
    
    marker.pose.position.x = 0 
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1
  
    return marker  

def mytopic_callback(msg):
    global status
    status = msg.hasil
    pos[0] = msg.position_x
    pos[1] = msg.position_y
    pos[2] = msg.position_z
    ori[0] = msg.orientation_x
    ori[1] = msg.orientation_y
    ori[2] = msg.orientation_z
    ori[3] = msg.orientation_w


def run():
   marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=100)
   markerbasics_object = makeBox()
   marker_objectlisher.publish(markerbasics_object)
  
rospy.init_node('marker_basic_node', anonymous=True)
rate = rospy.Rate(100) 
if __name__ == '__main__':
    try:
          while not rospy.is_shutdown():
              print("position", pos)
              print("orientation", ori)
              print("status", status)
              print("======================================================")
              run()
              rate.sleep()
    except rospy.ROSInterruptException:
        pass