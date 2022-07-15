#!/usr/bin/env python3

import rospy
import math
import numpy as np
import PyKDL as kdl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import time


#meter 
link1 = 17.7209
link2 = 4.9194
link3 = 20.7906
link4 = 7.8296

link = [link1, link2, link3,link4]

yaw = 45
yaw = np.radians(yaw)
cnt = 0  
#target
target = [0.222014,    0.53857632, -0.1671813]
        

def RX(yaw):
    return np.array([[1, 0, 0], 
                     [0, math.cos(yaw), -math.sin(yaw)], 
                     [0, math.sin(yaw), math.cos(yaw)]])   

def RY(delta):
    return np.array([[math.cos(delta), 0, math.sin(delta)], 
                     [0, 1, 0], 
                     [-math.sin(delta), 0, math.cos(delta)]])

def RZ(theta):
    return np.array([[math.cos(theta), -math.sin(theta), 0], 
                     [math.sin(theta), math.cos(theta), 0], 
                     [0, 0, 1]])

def TF(rot_axis=None, q=0, dx=0, dy=0, dz=0):
    if rot_axis == 'x':
        R = RX(q)
    elif rot_axis == 'y':
        R = RY(q)
    elif rot_axis == 'z':
        R = RZ(q)
    elif rot_axis == None:
        R = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])
    
    T = np.array([[R[0,0], R[0,1], R[0,2], dx],
                  [R[1,0], R[1,1], R[1,2], dy],
                  [R[2,0], R[2,1], R[2,2], dz],
                  [0, 0, 0, 1]])
    return T
    
def FK(angle, link):
    base = TF()
    T0 = TF('y', q = angle[0], dx = 0.0034, dy = 17.7209)
    T0_0 = base.dot(T0)
    T1 = TF('x', q = angle[1], dx = 2.506, dy = 4.9194, dz = -2.6717)
    T1_0 = T0_0.dot(T1)
    
    T2 = TF('y', q = angle[2], dx = -0.79, dy = -2.45, dz = -20.7906)
    T2_1 = T1_0.dot(T2)
    
    T3 = TF('z', q = angle[3],dx =-1.55, dy = 2.45, dz = -7.8296)
    T3_2 = T2_1.dot(T3)

    T4 = TF(dx = 0.1081, dz = -22.8470)
    T4_2 = T3_2.dot(T4)
    return base, T0_0, T1_0, T2_1, T4_2
    

def run():
    global cnt
    lb = np.array([(-np.pi, -np.radians(10), -(np.radians(160)) , 0)])
    ub = np.array([(np.radians(60),np.pi/2 , 0 , np.pi)])


    angle = np.random.uniform(low=lb, high=ub, size = 4)
    #angle = [-1.40792264, -0.06955575, -1.21595217, -np.radians(0)]
    
    p0, base, p1, p2, p3 = FK(angle,link)
    f_result = kdl.Frame(kdl.Rotation(p3[0,0], p3[0,1], p3[0,2],
                                    p3[1,0], p3[1,1], p3[1,2],
                                    p3[2,0], p3[2,1], p3[2,2]),
                        kdl.Vector(p3[0,3], p3[1,3], p3[2,3]))
    [qx, qy, qz, w] = f_result.M.GetQuaternion()
 #   print("end effector", p3[:3,3])
 #   print("rpy", drx,dry,drz)
 #   print("=============================================")
    pub = rospy.Publisher('pose_random', PoseStamped, queue_size = 1)
    msg = PoseStamped()
    msg.pose.position.x = p3[0,3]/100
    msg.pose.position.y = p3[1,3]/100  
    msg.pose.position.z = (p3[2,3]+27.6882)/100
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = w
    rospy.loginfo(msg)
    #print("angle", angle)
   # cnt+=1
    #if (cnt <= 1010):
    pub.publish(msg)
    #    print("cnt",cnt)
    #else:
    #    print("stop publish")
    time.sleep(1)
    
def main():
    global target, angle, link
    rospy.init_node('forward_random', anonymous=True)     
    rate = rospy.Rate(1)    
    #rospy.Subscriber('robotis/set_joint_states/', JointState, mytopic_callback)
    while not rospy.is_shutdown():
        run()
        rate.sleep()
    #rospy.spin()

    

   
if __name__ == "__main__":
    main()
    
    
    
    
    

