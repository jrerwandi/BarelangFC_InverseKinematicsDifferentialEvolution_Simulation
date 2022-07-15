#!/usr/bin/env python3

import math
import numpy as np
from numpy import array as a
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import PyKDL as kdl
from time import time, sleep
from numpy.lib.function_base import angle
import rospy
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

def draw_axis(ax, scale=0.5, A=np.eye(4), style='-'):
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
    #print (f_target.Inverse(), f_diff)

    [dx, dy, dz] = f_diff.p
    #print ([dx, dy, dz])
    [drz, dry, drx] = f_diff.M.GetEulerZYX()
    #print ([drx, dry, drz])
    error = np.sqrt(dx**2 + dy**2 + dz**2 + drx**2 + dry**2 + drz**2)
    error_list = [dx, dy, dz, drx, dry, drz]
    return error, error_list


def TwoPoints(t, P1, P2):
        
        if not isinstance(P1, np.ndarray) or not isinstance(P2, np.ndarray):
            raise TypeError('Points must be an instance of the numpy.ndarray!')
        if not isinstance(t, (int, float)):
            raise TypeError('Parameter t must be an int or float!')

        Q1 = (1 - t) * P1 + t * P2
        return Q1

def Points(t, points):
   
    newpoints = []
    #print("points =", points, "\n")
    for i1 in range(0, len(points) - 1):
        #print("i1 =", i1)
        #print("points[i1] =", points[i1])

        newpoints += [TwoPoints(t, points[i1], points[i1 + 1])]
        #print("newpoints  =", newpoints, "\n")
    return newpoints

def Point(t, points):
 
    newpoints = points
    #print("newpoints = ", newpoints)
    while len(newpoints) > 1:
        newpoints = Points(t, newpoints)
        #print("newpoints in loop = ", newpoints)

    #print("newpoints = ", newpoints)
    #print("newpoints[0] = ", newpoints[0])
    return newpoints[0]

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

def main(t_values, target, yawt):
    # Konfigurasi link
    global angle
    angle = []
    CROTCH_TO_HIP = 1.05 # Jarak croth ke hip
    UPPER_HIP = 0.755# Jarak hip yaw ke hip roll pitch
    HIP_TO_KNEE = 2.75#0.2215 # Panjang link upper leg
    KNEE_TO_ANKLE = 2.75#0.2215 # Panjang link lower leg
    ANKLE_TO_SOLE = 0.755#0.053 # Jarak ankle ke sole
    UP_KNEE_TO_DOWN_KNEE = 0 #jarak joint pada lutut
    
    A = HIP_TO_KNEE
    B = KNEE_TO_ANKLE
    D = CROTCH_TO_HIP
    a=-1
    frame = np.eye(4) * len(target[0])
    Fsole = np.eye(4)
    #print(frame)
    #print (yaw)
    for t in t_values:
        sleep(0.04)
        xyz = target
        #print(xyz[5,2])
        
        a = a + 1
        print (a)
        
        # Input target
        x_from_base = xyz[a,0]
        y_from_base = xyz[a,1]
        z_from_base = xyz[a,2]
        
        #print (yawt [a,1])
        yaw = yawt[a,1]
        #print("ololollol",x_from_base)
        # frame target
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
        #print xb, yb, z_from_hip
        # Inverse Kinematic
        # Mencari jarak Target ke Hip (C)
        C = np.sqrt(x_from_hip**2+y_from_hip**2+z_from_hip**2)
        #print("C =", C)

        # Mencari sudut Knee
        q4 = -np.arccos(((A**2+B**2)-C**2)/(2*A*B))
        q41 = q4/2
        q42 = q41
        #print("q4 = ",np.degrees(q4))
        qKnee = q4+np.radians(180)
        qKneeDown = -(np.radians(90) + q42)
        qKneeUp = -(np.radians(90) + q41)
        #print("qup = ",np.degrees(qkneeUp))
        #print("q42 . qkneedon : ",np.degrees(q42), np.degrees(qKneeDown))
        # Mencari sudut ankle pitch dan hip pitch
        q3 = (qKnee)/2
        #print("q3 = ",np.degrees(q3))
        qx = np.arcsin(x_from_hip/C)
        #print("qx =", np.degrees(qx))
        qHipPitch = -(q3+qx)
        
        q5 = q3
        qz = (np.sqrt(y_from_hip**2+z_from_hip**2))
        #print("qz :", np.degrees(qz))
        qAnklePitch = ((np.radians(180)-(np.arctan2(x_from_hip, np.sign(z_from_hip)*qz))) - q3)
        qHipYaw = yaw
        qHipRoll = np.arctan2(y_from_hip, np.sign(z_from_hip)*z_from_hip)
        qAnkleRoll = -qHipRoll

        '''
        print("Q hip Yaw 	: ", np.degrees(qHipYaw))
        print("Q hip Roll 	: ", np.degrees(qHipRoll))
        print("Q hip Pitch 	: ",np.degrees(qHipPitch))
        print("Q Knee UP	: ",np.degrees(qKneeUp))
        print("Q Knee Down 	: ",np.degrees(qKneeDown))
        print("Q ankle Pitch 	: ", np.degrees(qAnklePitch))
        print("Q ankle Roll 	: ", np.degrees(qAnkleRoll))
        '''

        angle = [-qHipYaw, qHipRoll, qHipPitch, qKneeUp, qKneeDown, -qAnklePitch, qAnkleRoll]

        # Forward kinematic
        base = TF()
        hip_yaw_from_base = TF(rot_axis='z', q=qHipYaw, dy=D)
        hip_yaw = base.dot(hip_yaw_from_base) #R01
        hip_roll_from_hip_yaw = TF(rot_axis='x', q=qHipRoll, dz=-UPPER_HIP)
        hip_roll = hip_yaw.dot(hip_roll_from_hip_yaw) #R12
        hip_pitch_from_hip_roll = TF(rot_axis='y', q=qHipPitch)
        hip_pitch = hip_roll.dot(hip_pitch_from_hip_roll) #R23
        hip = hip_pitch # hip frame
        knee_from_hip = TF(rot_axis='y', q=-qKneeUp, dz=-A)
        knee_up = hip.dot(knee_from_hip) #R34
        kneeDown_from_up = TF(rot_axis='y', q=-qKneeDown, dz=-UP_KNEE_TO_DOWN_KNEE)
        knee_Down = knee_up.dot(kneeDown_from_up) 
        ankle_pitch_from_knee = TF(rot_axis='y', q=qAnklePitch, dz=-B)
        ankle_pitch = knee_Down.dot(ankle_pitch_from_knee) #R45
        ankle_roll_from_ankle_pitch = TF(rot_axis='x', q=qAnkleRoll)
        ankle_roll = ankle_pitch.dot(ankle_roll_from_ankle_pitch) #R56
        ankle = ankle_roll
        sole_from_ankle = TF(dz=-ANKLE_TO_SOLE)
        sole = ankle.dot(sole_from_ankle) # sole frame R67
       
        f_result = kdl.Frame(kdl.Rotation(sole[0,0], sole[0,1], sole[0,2],
                                        sole[1,0], sole[1,1], sole[1,2],
                                        sole[2,0], sole[2,1], sole[2,2]),
                            kdl.Vector(sole[0,3], sole[1,3], sole[2,3]))

        error,errorList = pose_error(f_target,f_result)
        '''
        print("Input Position")
        print("x :", x_from_base)
        print("y :", y_from_base)
        print("z :", z_from_base)
        print("yaw :", np.degrees(yaw))
        print("Solution")
        print("x :", sole[0,3])
        print("y :", sole[1,3])
        print("z :", sole[2,3])
        '''
        #print("r :", np.sqrt(ankle[0,3]**2+ankle[1,3]**2+ankle[2,3]**2))
        print("ERROR :", error)
        print("=====================")
        #print("ERROR LIST :", errorList)
        
        Fsole = np.concatenate((Fsole, sole))
        publish()
        
    soleList = Fsole
    #angleList = angle
    N = 4
    #M = 7
    Fsole = [soleList[n:n+N] for n in range(0, len(soleList), N)]
    #angle = [angleList[n:n+M] for n in range(0, len(angleList), M)]
    #print(angle[3])
    return frame,Fsole
    

def Curve(t_values, points):
   
    if not hasattr(t_values, '__iter__'):
        raise TypeError("`t_values` Must be an iterable of integers or floats, of length greater than 0 .")
    if len(t_values) < 1:
        raise TypeError("`t_values` Must be an iterable of integers or floats, of length greater than 0 .")
    if not isinstance(t_values[0], (int, float)):
        raise TypeError("`t_values` Must be an iterable of integers or floats, of length greater than 0 .")

    curve = np.array([[0.0] * len(points[0])])
    for t in t_values:
        #print("curve                  \n", t)
        #print("Point(t, points) \n", Point(t, points))

        curve = np.append(curve, [Point(t, points)], axis=0)
        #main(curve)    
        #print("curve after            \n", curve, "\n--- --- --- --- --- --- ")
    curve = np.delete(curve, 0, 0)
    
    #print("curve final            \n", curve[2], "\n--- --- --- --- --- --- ")
    return curve

points_set_1 = a([[0, 1.05, -6.6], [0, 1.05, -5.5], [1.4, 1.05, -5.5], [1.4, 1.05, -6.6]])
t_points = np.arange(0, 1, 0.01)
curve_set_1 = Curve(t_points, points_set_1)
tyaw = np.radians(30)
time = 1
test = a([[0, 0], [0, tyaw], [time, 0], [time, tyaw]])
test_set_1 = Curve(t_points, test)
#fig = plt.figure(2)
#plt.plot(test_set_1[:, 0], test_set_1[:, 1])
#plt.plot(test[:, 0], test[:, 1], 'o:')

frame, sole = main(t_points, curve_set_1, test_set_1 )

fig = plt.figure(1)
ax = fig.add_subplot(projection='3d')
ax.set_xlim(0,0.5)
ax.set_ylim(0.8,1.3)
ax.set_zlim(-6.65,-6.15)
b = 0
for t in t_points:
    b = b+1
    draw_axis(ax,scale=0.05,A=sole[b])

ax.plot(curve_set_1[:, 0], curve_set_1[:, 1], curve_set_1[:, 2])
ax.plot(points_set_1[:, 0], points_set_1[:, 1], points_set_1[:, 2], 'o:')
plt.show()     
