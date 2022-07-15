#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from std_msgs.msg import Float64
from IK_Arm_Solver_DE.msg import jr
####1000 marker random joint   
def makeBox(ep,er,p,q): 
    p = np.asarray(p)  
    q = np.asarray(q)  
    ep = np.asarray(ep)  
    er = np.asarray(er)  
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.CUBE
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    
    if (ep > 1 or er > 0.1):
       print("merah")
       marker.color.r = 1
       marker.color.g = 0
       marker.color.b = 0
    else :
       print("hijau")
       marker.color.r = 0
       marker.color.g = 1
       marker.color.b = 0
    marker.color.a = 1.0


    marker.pose.position.x = p[0]
    marker.pose.position.y = p[1]
    marker.pose.position.z = p[2]

    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
  
    return marker  

def mytopic_callback(msg):
    print ("error posisi:", str(msg.err_pos))
    print ("error rotasi:", str(msg.err_rot))
    print ("posisi:", str(msg.position))
    print ("quaternion:", str(msg.quaternion))
    markerbasics_object = makeBox(msg.err_pos, msg.err_rot, msg.position, msg.quaternion)
    marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
            marker_objectlisher.publish(markerbasics_object)
            rate.sleep()
 
def run():
   rospy.init_node('marker_basic_node', anonymous=True)
   rospy.Subscriber('chatter', jr, mytopic_callback)
   
   rospy.spin()
   
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
