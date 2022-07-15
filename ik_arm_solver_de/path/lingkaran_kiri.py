#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import numpy as np

rospy.init_node('by_path', anonymous=True)
path_pub = rospy.Publisher('trajectory', Path, queue_size = 10)
pose_pub = rospy.Publisher('pose', PoseStamped, queue_size = 10)
rate = rospy.Rate(1)
hehe = False
def run():	
	global hehe
	f = 0
	seq = 0
	path = Path()
	path.header.stamp = rospy.Time.now()
	path.header.frame_id="base_link"
	for i in range(100):
		
		y = 0.07 * np.cos(f + i / 100.0 * 2 * np.pi)
		z = 0.07 * np.sin(f + i / 100.0 * 2 * np.pi)
		poseSt = PoseStamped()	
		poseSt.header.stamp = rospy.Time.now()
		poseSt.header.frame_id="base_link"
		poseSt.header.seq = seq
		poseSt.pose.position.x = 0.4
		poseSt.pose.position.y = y+0.25
		poseSt.pose.position.z = z+0.1
		poseSt.pose.orientation.x = 0
		poseSt.pose.orientation.y = 0
		poseSt.pose.orientation.z = 0
		poseSt.pose.orientation.w = 1
		rospy.loginfo(poseSt)
		pose_pub.publish(poseSt)
		

		path.poses.append(poseSt)
		if (i >= 98):
			hehe = True
		if hehe:
			time.sleep(1)

	rospy.loginfo(path)
	seq +=1
	f +=1
	print("====================")
	path_pub.publish(path)
			
if __name__ == '__main__':
	while not rospy.is_shutdown():
		run()
		
