#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

rospy.init_node('by_path', anonymous=True)
path_pub = rospy.Publisher('trajectory', Path, queue_size = 10)
pose_pub = rospy.Publisher('pose', PoseStamped, queue_size = 10)
rate = rospy.Rate(1)
start_delay = False #gambar pathnya dulu, setelah path muncul delay untuk publish aktif
move_y = True
point_y = 0.01
point_z = 0.01
y = 0.2
z = 0.2

def run():	
	global start_delay,move_y,point_y,point_z,y,z
	seq = 0
	path = Path()
	path.header.stamp = rospy.Time.now()
	path.header.frame_id="base_link"
	for point in range(50):
		if(move_y):
			y += point_y
			if (point >= 49):
				start_delay = True
			if start_delay:
				time.sleep(1)
			if (y <= 0.2 or y >= 0.3):
				point_y *= -1
				move_y = False		
		else:
			z += point_z
			if (point >= 49):
				start_delay = True
			if start_delay:
				time.sleep(1)
			if (z >= 0.3 or z <= 0.2):
				point_z *= -1
				move_y = True
		if (y >= 0.3):
			y = 0.3
		elif (y <= 0.2):
			y = 0.2
		if (z >= 0.3):
			z = 0.3
		elif (z <= 0.2):
			z = 0.2
			
		poseSt = PoseStamped()	
		poseSt.header.stamp = rospy.Time.now()
		poseSt.header.frame_id="base_link"
		poseSt.header.seq = seq
		poseSt.pose.position.x = 0.4
		poseSt.pose.position.y = y
		poseSt.pose.position.z = z
		poseSt.pose.orientation.x = 0
		poseSt.pose.orientation.y = 0
		poseSt.pose.orientation.z = 0
		poseSt.pose.orientation.w = 1
		rospy.loginfo(poseSt)
		pose_pub.publish(poseSt)
		

		path.poses.append(poseSt)
	rospy.loginfo(path)
	seq +=1
	print("====================")
	path_pub.publish(path)
	rate.sleep()
			
if __name__ == '__main__':
	while not rospy.is_shutdown():
		run()
		
