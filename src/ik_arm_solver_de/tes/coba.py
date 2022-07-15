#!/usr/bin/env python3
import rospy 
import rospkg 
from gazebo_msgs.msg import LinkState 
from gazebo_msgs.srv import SetLinkState


def main():
    rospy.init_node('set_pose')

    state_msg = LinkState()
    state_msg.link_name = 'humanoid::r_ank_roll_link'
    state_msg.pose.position.x = 0.05
    state_msg.pose.position.y = 0
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0

    rospy.wait_for_service('/gazebo/set_link_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        resp = set_state( state_msg )

    except rospy.ServiceException:
        print ("Service call failed: ")#%s" % e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
