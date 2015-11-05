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
import math

d=26.5
r=10

def scout_pub_xyt(motion_pub,x=0,y=0,theta=0):
    smm_msg=smm()
    smm_msg.enable=True


    ang_ini = math.acos(y/x) * 180/(2*math.pi) #Initial orientation of the robot

    if (ang_ini<0):
        vel_ang=-100 #ter atencao a qual e negativo e qual e positivo
    else:
        vel_ang = 100

    #First rotation of the robot
    smm_msg.velocity_left= d/r*vel_ang
    smm_msg.velocity_right= d/r*vel_ang
    motion_pub.publish(smm_msg)
    time.sleep(5)
    #Linear movement of the robot



    #if x==0 and y==0:
    #    smm_msg.velocity_left=0
    #    smm_msg.velocity_right=0
    #else:
    #    smm_msg.velocity_left=default_scout_vel
    #    smm_msg.velocity_right=-default_scout_vel

    motion_pub.publish(smm_msg)



if __name__ == '__main__':
    try:
        rospy.init_node('scout_control', anonymous=True)
        motion_pub = rospy.Publisher('/scout/motion', smm, queue_size=1)
        time.sleep(5)
        scout_pub_xyt(motion_pub,1,0,0)
        while not rospy.is_shutdown():
            time.sleep(0.5)
            scout_pub_xyt(motion_pub,1,1,0)
    except rospy.ROSInterruptException:
        pass
