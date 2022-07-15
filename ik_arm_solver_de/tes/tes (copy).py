#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point 
import numpy as np
from std_msgs.msg import Float64
from beginner_tutorials.msg import jr
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
   
def makeBox(ep,er,msg): 
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


    marker.pose.position.x = msg.pose.position.x
    marker.pose.position.y = msg.pose.position.y
    marker.pose.position.z = msg.pose.position.z

    marker.pose.orientation.x = msg.pose.orientation.x
    marker.pose.orientation.y = msg.pose.orientation.y
    marker.pose.orientation.z = msg.pose.orientation.z
    marker.pose.orientation.w = msg.pose.orientation.w
  
    return marker  

def mytopic_callback(msg):
#    print ("error posisi:", str(msg.err_pos))
#    print ("error rotasi:", str(msg.err_rot))
#    print ("posisi:", str(msg.pose.position))
#    print ("quaternion:", str(msg.pose.quaternion))
    print("data", msg)
#    print("data", msg.pose.position.x)
   # print("x", msg.poses.position.x)
    markerbasics_object = makeBox(1, 1, msg)
    marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
            marker_objectlisher.publish(markerbasics_object)
            rate.sleep()
 
def run():
   rospy.init_node('marker_basic_node', anonymous=True)
   rospy.Subscriber('pose', PoseStamped, mytopic_callback)
   
   rospy.spin()
   
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
