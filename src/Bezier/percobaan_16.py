#!/usr/bin/env python3

import math
import numpy as np
from numpy import array as a, degrees
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import PyKDL as kdl
from time import time, sleep
from numpy.lib.function_base import angle
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

def set_label(ax, min_scale=-10, max_scale=10):
    ax.set_xlim(min_scale, max_scale)
    ax.set_ylim(min_scale, max_scale)
    ax.set_zlim(min_scale, max_scale)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

def draw_axis(ax, scale=0.02, A=np.eye(4), style='-'):
    xaxis = np.array([[0, 0, 0, 1], [scale, 0, 0, 1]]).T
    yaxis = np.array([[0, 0, 0, 1], [0, scale, 0, 1]]).T
    zaxis = np.array([[0, 0, 0, 1], [0, 0, scale, 1]]).T
    xc = A.dot(xaxis)
    yc = A.dot(yaxis)
    zc = A.dot(zaxis) 
    ax.plot(xc[0,:], xc[1,:], xc[2,:], 'r' + style)
    ax.plot(yc[0,:], yc[1,:], yc[2,:], 'g' + style)
    ax.plot(zc[0,:], zc[1,:], zc[2,:], 'b' + style)
   
def RX(alpha):
    return np.array([[1, 0, 0], 
                     [0, math.cos(alpha), -math.sin(alpha)], 
                     [0, math.sin(alpha), math.cos(alpha)]])   

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

def draw_links(ax, origin_frame=np.eye(4), target_frame=np.eye(4)):
    x = [origin_frame[0,3], target_frame[0,3]]
    y = [origin_frame[1,3], target_frame[1,3]]
    z = [origin_frame[2,3], target_frame[2,3]]
    ax.plot(x, y, z, 'k')

def pose_error(f_target, f_result):
    f_diff = f_target.Inverse() * f_result
    [dx, dy, dz] = f_diff.p
    [drz, dry, drx] = f_diff.M.GetEulerZYX()
    error = np.sqrt(dx**2 + dy**2 + dz**2 + drx**2 + dry**2 + drz**2)
    error_list = [dx, dy, dz, drx, dry, drz]
    return error, error_list

def publish():
    pub1 = rospy.Publisher('/humanoid/l_hip_yaw_joint_position_controller/command', Float64, queue_size=100)
    pub2 = rospy.Publisher('/humanoid/l_hip_roll_joint_position_controller/command', Float64, queue_size=100)
    pub4 = rospy.Publisher('/humanoid/l_ank_pitch_back_joint_position_controller/command', Float64, queue_size=100)
    pub3 = rospy.Publisher('/humanoid/l_knee_joint_position_controller/command', Float64, queue_size=100)
    pub5 = rospy.Publisher('/humanoid/l_ank_roll_joint_position_controller/command', Float64, queue_size=100)
    rospy.init_node('joint_positions_node', anonymous=True)
    
    
    pub1.publish(angle[0])
    pub2.publish(angle[1])
    pub3.publish(angle[2])
    pub4.publish(angle[5])
    pub5.publish(angle[6])

def FK(angle):
    # Forward kinematic
    #fig = plt.figure() # Deklarasi matplot
    #ax = fig.add_subplot(111, projection='3d')
    
    base = TF()
    hip_yaw_from_base = TF(rot_axis='z', q=angle[0], dy=0.105)
    hip_yaw = base.dot(hip_yaw_from_base) #R01
    hip_roll_from_hip_yaw = TF(rot_axis='x', q=angle[1], dz=-0.0625)
    hip_roll = hip_yaw.dot(hip_roll_from_hip_yaw) #R12
    hip_pitch_from_hip_roll = TF(rot_axis='y', q=-angle[2])
    hip_pitch = hip_roll.dot(hip_pitch_from_hip_roll) #R23
    hip = hip_pitch # hip frame
    knee_from_hip = TF(rot_axis='y', q=angle[2], dz=-0.275)
    knee_up = hip.dot(knee_from_hip) #R34
    kneeDown_from_up = TF(rot_axis='y', q=angle[3], dz=-0.16)
    knee_Down = knee_up.dot(kneeDown_from_up) 
    ankle_pitch_from_knee = TF(rot_axis='y', q=-angle[3], dz=-0.275)
    ankle_pitch = knee_Down.dot(ankle_pitch_from_knee) #R45
    ankle_roll_from_ankle_pitch = TF(rot_axis='x', q=-angle[1])
    ankle_roll = ankle_pitch.dot(ankle_roll_from_ankle_pitch) #R56
    ankle = ankle_roll
    sole_from_ankle = TF(dz=-0.0475)
    sole = ankle.dot(sole_from_ankle) # sole frame R67
    '''
    draw_axis(ax, A=base)
    draw_links(ax, origin_frame=base, target_frame=hip_yaw)
    draw_axis(ax, A=hip_yaw)
    draw_links(ax, origin_frame=hip_yaw, target_frame=hip)
    draw_axis(ax, A=hip)
    draw_links(ax, origin_frame=hip, target_frame=knee_up)
    draw_axis(ax, A=knee_up)
    draw_links(ax, origin_frame=knee_up, target_frame=knee_Down)
    draw_axis(ax, A=knee_Down)
    draw_links(ax, origin_frame=knee_Down, target_frame=ankle)
    draw_axis(ax, A=ankle)
    draw_links(ax, origin_frame=ankle, target_frame=sole)
    draw_axis(ax, A=sole)
    '''
    f_result = kdl.Frame(kdl.Rotation(sole[0,0], sole[0,1], sole[0,2],
                                    sole[1,0], sole[1,1], sole[1,2],
                                    sole[2,0], sole[2,1], sole[2,2]),
                        kdl.Vector(sole[0,3], sole[1,3], sole[2,3]))
    xyz = [sole[0,3],sole[1,3],sole[2,3],angle[0]]

    return xyz, f_result

def main(t_values):
    # Konfigurasi link
    global angle
    angle = []
    CROTCH_TO_HIP = 0.105    # Jarak croth ke hip
    UPPER_HIP = 0.0625       # Jarak hip yaw ke hip roll pitch
    HIP_TO_KNEE = 0.275      # Panjang link upper leg
    KNEE_TO_ANKLE = 0.275    # Panjang link lower leg
    ANKLE_TO_SOLE = 0.0475   # Jarak ankle ke sole
    UP_KNEE_TO_DOWN_KNEE = 0.16 #jarak joint pada lutut
    
    A = HIP_TO_KNEE
    B = KNEE_TO_ANKLE
    D = CROTCH_TO_HIP
    a=-1
    b = 0
    fig = plt.figure()
    for t in t_values:
        #sleep(0.02)
        ub = [0.4,0,3.8,0.8]
        lb = [-0.4,0,0,0]
        randomangle = np.random.uniform(low = lb, high = ub, size = 4)
        #print (randomangle)
        xyz,frandom = FK(randomangle) 
        #print(xyz)
        a = a + 1
        print (a)
        
        # Input target
        x_from_base = xyz[0]
        y_from_base = xyz[1]
        z_from_base = xyz[2] 

        yaw = xyz[3]
        
        f_target = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw), kdl.Vector(x_from_base, y_from_base, z_from_base))
    
        # Jarak target ke hip
        x_from_hip = (x_from_base - 0)
        y_from_hip = (y_from_base - CROTCH_TO_HIP)
        z_from_hip = (z_from_base + UPPER_HIP + ANKLE_TO_SOLE + UP_KNEE_TO_DOWN_KNEE)

        # jarak target ke hip dengan yaw
        xa = x_from_hip
        ya_1 = xa*np.tan(yaw)
        xb_1 = xa / np.cos(yaw)
        beta = np.radians(90) - yaw
        ya_2 = (y_from_hip-ya_1)
        yb = ya_2 * np.sin(beta)
        xb_2 = yb / np.tan(beta) #!90
        xb = xb_1 + xb_2
        #print ya_1, ya_2, yb, xb_1, xb_2, xb, beta

        x_from_hip = xb
        y_from_hip = yb
        z_from_hip = z_from_hip
        #print (xb, yb, z_from_hip)
        # Inverse Kinematic
        # Mencari jarak Target ke Hip (C)
        C = np.sqrt(x_from_hip**2+y_from_hip**2+z_from_hip**2)
        z_from_hip_roll = z_from_hip - 0.16
        qHipRoll = np.arctan2(y_from_hip, np.sign(z_from_hip_roll)*z_from_hip_roll)
        qAnkleRoll = -qHipRoll
        qroll = qHipRoll

        # Mencari sudut Knee
        q4 = -np.arccos(((A**2+B**2)-C**2)/(2*A*B))
        qrol2 = np.radians(180)-(np.radians(90)+qHipRoll) 
        qKnee = q4+np.radians(180)
        
        PUSING = (0.1*np.cos(qrol2))
        PUSING2 = (0.075*np.sin(qroll))
        #print(PUSING2,PUSING)
        #z_from_hip = z_from_hip - PUSING
        x_from_hip = x_from_hip + PUSING2
        # Mencari sudut ankle pitch dan hip pitch
        q3 = qKnee/2
        qx = np.arcsin((x_from_hip)/C)
        qHipPitch = -(q3+qx)
        
        q5 = q3
        qz = (np.sqrt(y_from_hip**2+z_from_hip**2))
        qAnklePitch = ((np.radians(180)-(np.arctan2(x_from_hip, np.sign(z_from_hip)*qz))) - q3)
        qHipYaw = yaw
        
        qKneeUp = qHipPitch 
        qKneeDown = qAnklePitch
        
        angle = [qHipYaw, qHipRoll, -qHipPitch, -qAnklePitch]
        #print(angle)
        xyz2,f_result = FK(angle)

        error,errorList = pose_error(f_target,f_result)
        '''
        print("C =", C)
        print("qroll",np.degrees(qroll))
        print("qroll2",np.degrees(qrol2))
        print("q3 = ",np.degrees(q3))
        print("qx =", np.degrees(qx))
        print("qz :", np.degrees(qz))
        
        print("Q hip Yaw 	: ", np.degrees(angle[0]))
        print("Q hip Roll 	: ", np.degrees(angle[1]))
        print("Q hip Pitch 	: ",angle[2])
        print("Q Knee UP	: ",angle[2])
        print("Q Knee Down 	: ",angle[3])
        print("Q ankle Pitch 	: ", angle[3])
        print("Q ankle Roll 	: ", angle[1])
        
        print("Input Position")
        print("x :", x_from_base)
        print("y :", y_from_base)
        print("z :", z_from_base)
        print("yaw :", np.degrees(yaw))
        
        print("Solution")
        print("x :", xyz2[0])
        print("y :", xyz2[1])
        print("z :", xyz2[2])
        
        print("ERROR :", error)
        print("=====================")
        #print("ERROR LIST :", errorList)
        '''
        
        if error < 0.01:
            plt.scatter(a,error)
            b+=1
            status = "IK_Solved"
        else:
            status = "IK_Error"
        #Fsole = np.concatenate((Fsole, sole))
        #publish()
        
    '''
    soleList = Fsole
    N = 4
    Fsole = [soleList[n:n+N] for n in range(0, len(soleList), N)]
    print(Fsole)
    '''
    return b
t_points = np.arange(0, 1, 0.001)
      
b = main(t_points)
print ("total berhasil",b)
#plt.show()    
