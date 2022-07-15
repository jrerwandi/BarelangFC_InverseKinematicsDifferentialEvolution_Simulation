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
def publish():
    pub1 = rospy.Publisher('/humanoid/l_hip_yaw_joint_position_controller/command', Float64, queue_size=100)
    pub2 = rospy.Publisher('/humanoid/l_hip_roll_joint_position_controller/command', Float64, queue_size=100)
    pub4 = rospy.Publisher('/humanoid/l_ank_pitch_back_joint_position_controller/command', Float64, queue_size=100)
    pub3 = rospy.Publisher('/humanoid/l_knee_joint_position_controller/command', Float64, queue_size=100)
    pub5 = rospy.Publisher('/humanoid/l_ank_roll_joint_position_controller/command', Float64, queue_size=100)
    rospy.init_node('joint_positions_node', anonymous=True)


    pub1.publish(0.0)
    pub2.publish(0.0)
    pub3.publish(0.0)
    pub4.publish(0.0)
    pub5.publish(0.0)

publish()