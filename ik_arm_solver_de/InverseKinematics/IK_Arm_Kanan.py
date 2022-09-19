#!/usr/bin/env python3

import math
import numpy as np
import PyKDL as kdl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from differentialEvolution import DE
from time import time, sleep
import time as tm
import rospy
from std_msgs.msg import Header, Float64, String
from sensor_msgs.msg import JointState
from ik_arm_solver_de.msg import jr
from geometry_msgs.msg import PoseStamped

start = time()
#cm

link1 = 17.7209
link2 = 4.9194
link3 = 20.7906
link4 = 7.8296

link = [link1, link2, link3,link4]
target = [40,20,4]
orientation = [0,0,0,0]
yaw = 0

def draw_axis(ax, scale=1.0, O=np.eye(4), style='-'):
    xaxis = np.array([[0, 0, 0, 1], [scale, 0, 0, 1]]).T
    yaxis = np.array([[0, 0, 0, 1], [0, scale, 0, 1]]).T
    zaxis = np.array([[0, 0, 0, 1], [0, 0, scale, 1]]).T
    xc = O.dot(xaxis)
    yc = O.dot(yaxis)
    zc = O.dot(zaxis) 
    ax.plot(xc[0,:], xc[1,:], xc[2,:], 'r' + style)
    ax.plot(yc[0,:], yc[1,:], yc[2,:], 'g' + style)
    ax.plot(zc[0,:], zc[1,:], zc[2,:], 'b' + style)
    
def draw_links(ax, origin_frame=np.eye(4), target_frame=np.eye(4)):
    x = [origin_frame[0,3], target_frame[0,3]]
    y = [origin_frame[1,3], target_frame[1,3]]
    z = [origin_frame[2,3], target_frame[2,3]]
    ax.plot(x, y, z, linewidth = 3)


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
    T0 = TF('y', q = angle[0], dx = 0.0034, dy = -17.7209)
    T0_0 = base.dot(T0)
    T1 = TF('x', q = angle[1], dx = 2.506, dy = -4.9194, dz = -2.6717)
    T1_0 = T0_0.dot(T1)
    
    T2 = TF('y', q = angle[2], dx = -0.79, dy = -2.45, dz = -20.7906)
    T2_1 = T1_0.dot(T2)
    
    T3 = TF('z', q = angle[3],dx =-1.55, dy = 2.45, dz = -7.8296)
    T3_2 = T2_1.dot(T3)

    T4 = TF(dx = 0.1081, dz = -22.8470)
    T4_3 = T3_2.dot(T4)
    return base, T0_0, T1_0, T2_1, T4_3
    
def obj_func (f_target, thetas, link):
    _,_,_,_,p = FK(thetas,link)
    f_result = kdl.Frame(kdl.Rotation(p[0,0], p[0,1], p[0,2],
                                      p[1,0], p[1,1], p[1,2],
                                      p[2,0], p[2,1], p[2,2]),
                         kdl.Vector(p[0,3], p[1,3], p[2,3]))

    f_diff = f_target.Inverse() * f_result
    
    [dx, dy, dz] = f_diff.p
    [drz, dry, drx] = f_diff.M.GetEulerZYX()
    
    error = np.sqrt(dx**2 + dy**2 + dz**2 + drz**2) #pilih yaw aja
    
    return error, thetas
    

def cekError(f_target, r):
    f_result = kdl.Frame(kdl.Rotation(r[0,0], r[0,1], r[0,2],
                                      r[1,0], r[1,1], r[1,2],
                                      r[2,0], r[2,1], r[2,2]),
                         kdl.Vector(r[0,3], r[1,3], r[2,3]))
    
    f_diff = f_target.Inverse() * f_result
    
    [dx, dy, dz] = f_diff.p
    [drz, dry, drx] = f_diff.M.GetEulerZYX()
    
    error = np.sqrt(dx**2 + dy**2 + dz**2 + drx**2 + dry**2 + drz**2)
    
    error_pos = np.sqrt(dx**2 + dy**2 + dz**2)
    error_rot = np.sqrt(drz**2)
    
    error_list = [dx, dy, dz, drx, dry, drz]
    
    return error, error_list, f_result, error_pos, error_rot


def publish():
    pub = rospy.Publisher('robotis/set_joint_states/', JointState, queue_size = 1)
    pub2 = rospy.Publisher('chatter', jr, queue_size = 1)
    status = "loading"
    if err_p > 1 or err_r > 0.1: 
        status = "IK_Error"
    else:
        status = "IK_Solved"
    data2 = jr()
    data2.hasil = status   
    data2.position_x = target[0]/100
    data2.position_y = target[1]/100
    data2.position_z = (target[2]/100) +  0.276882
    data2.orientation_x = orientation[0]
    data2.orientation_y = orientation[1]
    data2.orientation_z = orientation[2]
    data2.orientation_w = orientation[3]
    
    data = JointState()
    data.header.stamp = rospy.Time.now()
    data.name = ['r_sho_pitch', 'r_sho_roll', 'r_el', 'r_el_yaw']
    data.position = [angle[0], angle[1], angle[2], angle[3]]
    
   # rospy.loginfo(data)

    target2 = [target[0], target [1], target[2]+27.6882]
    print("============================================")
    print("target", target2) 
    print("============================================")
    print("error pos", err_p)
    print("error rot", err_r)
    print("angle", angle)
    pub.publish(data)
    pub2.publish(data2)

         
def run():
    global target, angle, link, orientation, err_r, err_p, yaw


    quer = kdl.Rotation.Quaternion(orientation[0], orientation[1], orientation[2],orientation[3])
    [yaw,_,_] = quer.GetEulerZYX()
  #  yaw = 90
   # yaw = np.radians(yaw)
    f_target = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw), kdl.Vector(target[0], target[1], target[2]))
    
    #jumlah yg di inisialisasi
    n_params = 4
    
    #batas bawah dan atas 
    lb = np.array([(-np.pi, -np.pi/2, -(np.radians(160)) , 0)])
    ub = np.array([(np.radians(60),np.radians(10) , 0 , np.pi)])    
    angle = []
    
    
    #inverse Kinematics
    err, angle = DE(obj_func, f_target, angle, link, n_params, lb, ub)
    
  
    #forward Kinematics
    p0, base, p1, p2, p3 = FK(angle,link)
    ulang = 0
    Done = False

    Cerror, err_list, f_r, err_p, err_r = cekError(f_target, p3)
    while ((err_p > 1 or err_r > 0.1) and Done == False):
       if ulang >= 0:
          Done = True
          print("+++++++++++++++++++++++++++++++++")
          print("IK ERROR")
          print("+++++++++++++++++++++++++++++++++")
       else:
          err, angle = DE(obj_func, f_target, angle, link, n_params, lb, ub)
          p0, base, p1, p2, p3 = FK(angle,link)
          Cerror, err_list, f_r, err_p, err_r = cekError(f_target, p3)
          ulang +=1
    
    Done = True
    

    [drz, dry, drx] = f_target.M.GetEulerZYX()
    [drz2, dry2, drx2] = f_r.M.GetEulerZYX()
    [qx, qy, qz, w] = f_r.M.GetQuaternion()
    #[x,y,z] = f_r.p
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")  
    print("hasil x", p3[0,3])
    print("hasil y", p3[1,3])
    print("hasil z", p3[2,3]+ 27.6882)

    
    print("target r p y", drx, dry, drz) 
    print("hasil r p y", drx2, dry2, drz2)     
    print("yaw hasil", drz2)
    print("yaw target", drz)
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")  

    publish()
    if (err_p <= 0.1 and err_r <= 0.01): 
        print("IK Solved")
    else:
        print("IK Error")
     
def mytopic_callback(msg):
    target [0] = msg.pose.position.x*100
    target [1] = msg.pose.position.y*100
    target [2] = (msg.pose.position.z - 0.276882) *100 #0.276882
    orientation [0] = msg.pose.orientation.x
    orientation [1] = msg.pose.orientation.y
    orientation [2] = msg.pose.orientation.z
    orientation [3] = msg.pose.orientation.w
    run()

def subs():  
    rospy.init_node('joint_positions_node', anonymous=True)        
    rospy.Subscriber('pose', PoseStamped, mytopic_callback)            
    rospy.Subscriber('pose_random', PoseStamped, mytopic_callback)
    rospy.spin()
    print("start")  
      
if __name__ == "__main__":
    subs()   

       
