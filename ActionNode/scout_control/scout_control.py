#!/usr/bin/env python

#export PYTHONPATH=$PYTHONPATH:/home/jmirandadealme/Documents/SistAut/HumanRobotInteraction/ros_scout-2.0/scout_msgs/src/

'''
Node has:
    Subscribers:
        - Get delta (x,y,theta) read from "/scout_commands"
            read: human_interf_msgs/ScoutCommandsMsg
        - Get odom current (x,y,theta) read from "/odom"
            read: ?

    Publishers:
        - Send state to "/scout_state"
            write: human_interf_msgs/ScoutStateMsg
        - Send commands to "/scout/motion"
            write: scout_msgs/ScoutMotionMsg



Notes:
    - Odometry is going to acumualte error.
'''

default_scout_vel = 1500


import rospy
from scout_msgs.msg import ScoutMotionMsg as smm
import time
def scout_pub_xyt(motion_pub,x=0,y=0,theta=0):
    smm_msg=smm()
    smm_msg.enable=True
    if x==0 :
        smm_msg.velocity_left=0
        smm_msg.velocity_right=0
    else:
        smm_msg.velocity_left=default_scout_vel
        smm_msg.velocity_right=-default_scout_vel
    motion_pub.publish(smm_msg)



if __name__ == '__main__':
    try:
        rospy.init_node('scout_control', anonymous=True)
        motion_pub = rospy.Publisher('/scout/motion', smm, queue_size=1)
        time.sleep(5)
        scout_pub_xyt(motion_pub,1,0,0)
        while not rospy.is_shutdown():
            time.sleep(0.5)
            scout_pub_xyt(motion_pub,0,0,0)
    except rospy.ROSInterruptException:
        pass
