#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

main (int argc, char **argv)
{
	ros::init (argc, argv, "by_path");

	ros::NodeHandle ph;
	ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);
	ros::Publisher path_pub2 = ph.advertise<geometry_msgs::PoseStamped>("pose",1, true);
	

	float f = 0.0, j = 0.01, l =0.01, x = 0.2,y =0.2,z;
	int k = 0, i = 0, p =1;
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		nav_msgs::Path path;
		geometry_msgs::PoseStamped this_pose_stamped;
		//nav_msgs::Path path;
		path.header.stamp=ros::Time::now();
		path.header.frame_id="base_link";
		for (uint32_t i = 0; i < 50; ++i)
		{
		if (k==0) {
			x += j;
			sleep(1);
			if (x <= 0.2 || x >= 0.3) {
				j *= -1;
				k = 1;
				
			}
		} else {
		        y += l;
		        sleep(1);
			if (y >= 0.3 || y <= 0.2) {
				l *= -1;
				k = 0;
			}
		}
		if (x >= 0.3) { x = 0.3;}
		else if (x <= 0.2) { x = 0.2;}
		if (y >= 0.3) { y = 0.3;}
		else if (y <= 0.2) { y = 0.2;}
		
			//float z = 0.1 * sin(f + i / 100.0f * 2 * M_PI);
			//float y = 0.1 * cos(f + i / 100.0f * 2 * M_PI);
			//float x = 0 * cos(f + i / 100.0f * 2 * M_PI);
				
			this_pose_stamped.pose.position.x = 0.4;
			this_pose_stamped.pose.position.y = x;
			this_pose_stamped.pose.position.z = y; //0,276882

			//geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(1);
			this_pose_stamped.pose.orientation.x = 0;
			this_pose_stamped.pose.orientation.y = 0;
			this_pose_stamped.pose.orientation.z = 0;
			this_pose_stamped.pose.orientation.w = 1;

			this_pose_stamped.header.stamp=ros::Time::now();;
			this_pose_stamped.header.frame_id="base_link";

			path.poses.push_back(this_pose_stamped);
			ROS_INFO("x = %f",this_pose_stamped.pose.position.x);
			ROS_INFO("y = %f",this_pose_stamped.pose.position.y);
			ROS_INFO("z = %f",this_pose_stamped.pose.position.z);
			ROS_INFO("qx = %f",this_pose_stamped.pose.orientation.x);
			ROS_INFO("qy = %f",this_pose_stamped.pose.orientation.y);
			ROS_INFO("qz = %f",this_pose_stamped.pose.orientation.z);
			ROS_INFO("w = %f",this_pose_stamped.pose.orientation.w);
			path_pub2.publish(this_pose_stamped);
		
		}
		printf("heheheheheheheeeeeeeeeeeeeeeeeeeee");
		path_pub.publish(path);
		//ros::spinOnce();               // check for incoming messages
		loop_rate.sleep();
		f += 1;
	}

	return 0;
}
