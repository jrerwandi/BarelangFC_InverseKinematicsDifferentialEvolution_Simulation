import math
import numpy as np
import PyKDL as kdl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d


#meter 

link1 = 17.7209
link2 = 4.9194
link3 = 23.46245+ 1.592805723
link4 = 30.68395+ 1.592805723
 
link = [link1/100, link2/100, link3/100,link4/100]

yaw = 45
yaw = np.radians(yaw)
   
#target
target = [0.222014,    0.53857632, -0.1671813]
        

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
    T0 = TF('y', q = angle[0], dy = link[0])
    T0_0 = base.dot(T0)
    T1 = TF('x', q = angle[1], dy = link[1])
    T1_0 = T0_0.dot(T1)
    
    T2 = TF('y', q = angle[2], dz = -link[2])
    T2_1 = T1_0.dot(T2)
    
    T3 = TF('z', q = angle[3], dz = -link[3])
    T3_2 = T2_1.dot(T3)

    
    return base, T0_0, T1_0, T2_1, T3_2
    

def main():
    global target, angle, link
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    fig.suptitle("Differential Evolution - Inverse Kinematics", fontsize=12)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


    angle = [-1.40792264, -0.06955575, -1.21595217, -np.radians(0)]
    
    
    
    
    #inverse Kinematics
    
  
    p0, base, p1, p2, p3 = FK(angle,link)
    f_result = kdl.Frame(kdl.Rotation(p3[0,0], p3[0,1], p3[0,2],
                                      p3[1,0], p3[1,1], p3[1,2],
                                      p3[2,0], p3[2,1], p3[2,2]),
                         kdl.Vector(p3[0,3], p3[1,3], p3[2,3]))
    [drz, dry, drx] = f_result.M.GetEulerZYX()
 #   print("angle", angle)
#    print("target", target)
    print("end effector", p3[:3,3])
    print("x", p3[0,3])
    print("y", p3[1,3])
    print("z", p3[2,3])

    print("rpy", drx,dry,drz)
#    print(p3.Inverse())
    #plot
    draw_axis(ax, scale=0.03, O=p0)
    draw_links(ax, origin_frame=p0, target_frame=base)
    draw_axis(ax, scale=0.03, O=base)
    draw_links(ax, origin_frame=base, target_frame=p1)
    draw_axis(ax, scale=0.03, O=p1)
    draw_links(ax, origin_frame=p1, target_frame=p2)
    draw_axis(ax, scale=0.03, O=p2)
    draw_links(ax, origin_frame=p2, target_frame=p3)
    draw_axis(ax, scale=0.03, O=p3)
    plt.show()
   
if __name__ == "__main__":
    main()
    
    
    
    
    

