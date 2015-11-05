#!/usr/bin/env python

#export PYTHONPATH=$PYTHONPATH:/home/jmirandadealme/Documents/SistAut/HumanRobotInteraction/ros_scout-2.0/scout_msgs/src/

'''
Node has:
    Subscribers:
        - Get delta (x,y,theta) read from "/scout_commands"
                read: geometry_msgs/Pose2D
        - Get odom current (x,y,theta) read from "/odom"
            read: nav_msgs/Odometry

    Publishers:
        - Send state to "/scout_state"
            write: human_interf_msgs/ScoutStateMsg
        - Send commands to "/scout/motion"
            write: scout_msgs/ScoutMotionMsg



Notes:
    - Odometry is going to acumualte error.
'''

default_scout_vel = 1500
d=26.5
r=10

import rospy
from scout_msgs.msg import ScoutMotionMsg as smm
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
import time
import math
import tf
import threading

rpy = [0,0,0]
scout_x = 0
scout_y = 0

scout_left_vel = 0
scout_right_vel = 0

command_x = 0
command_y = 0
command_t = 0

x_at_command = 0
y_at_command = 0
t_at_command = 0

my_lock = threading.Lock()

def scout_publisher(motion_pub):
    global my_lock
    while(True):
        smm_msg=smm()
        smm_msg.enable=True
        my_lock.acquire()
        smm_msg.velocity_left = scout_left_vel
        smm_msg.velocity_right = scout_right_vel
        my_lock.release()
        motion_pub.publish(smm_msg)
        time.sleep(0.25)

'''
def scout_pub_xyt(motion_pub,x=0,y=0,theta=0):
    smm_msg=smm()
    smm_msg.enable=True

    if x!=0:
        ang_ini = math.acos(y/x) * 180/(2*math.pi) #Initial orientation of the robot
    else:
        ang_ini = y/(abs(y))*90

    if (ang_ini<0):
        vel_ang=-100
    else:
        vel_ang = 100

    #First rotation of the robot
    smm_msg.velocity_left= d/r*vel_ang
    smm_msg.velocity_right= d/r*vel_ang
    for i in range(10):
        motion_pub.publish(smm_msg)
        time.sleep(0.5)
    #Linear movement of the robot
    smm_msg.velocity_left=default_scout_vel
    smm_msg.velocity_right=-default_scout_vel
    for i in range(10):
        motion_pub.publish(smm_msg)
        time.sleep(0.5)
    #Second rotation of the robot
    ang2=theta-ang_ini
    if (ang2<0):
        vel_ang=-100
    else:
        vel_ang = 100

    smm_msg.velocity_left= d/r*vel_ang
    smm_msg.velocity_right= d/r*vel_ang
    for i in range(10):
        motion_pub.publish(smm_msg)
        time.sleep(0.5)

    motion_pub.publish(smm_msg)
'''

def odom_process(data):
    quaternion=[0,0,0,0]
    global rpy
    global scout_x
    global scout_y
    global my_lock
    quaternion[0]=data.pose.pose.orientation.x
    quaternion[1]=data.pose.pose.orientation.y
    quaternion[2]=data.pose.pose.orientation.z
    quaternion[3]=data.pose.pose.orientation.w
    my_lock.acquire()
    scout_x = data.pose.pose.position.x
    scout_y = data.pose.pose.position.y
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    my_lock.release()

def command_process(data):
    global command_x
    global command_y
    global command_t
    global x_at_command
    global y_at_command
    global t_at_command
    global scout_left_vel
    global scout_right_vel
    global my_lock

    my_lock.acquire()

    # Store the commands
    command_x = data.x
    command_y = data.y
    command_t = data.theta


    # Store the scout info at command receival
    x_at_command = scout_x
    y_at_command = scout_y
    t_at_command = rpy[2]

    # Generate velocity information
    scout_left_vel = 200
    scout_right_vel = 200

    my_lock.release()

    rospy.loginfo('command received (x,y,t) = ( ' + str(data.x) + ', ' + str(data.y) + ', ' + str(data.theta) + ')')


if __name__ == '__main__':
    try:

        rospy.init_node('scout_control', anonymous=True)
        motion_pub = rospy.Publisher('/scout/motion', smm, queue_size=1)
        rospy.Subscriber('/odom', Odometry, odom_process)
        rospy.Subscriber('/scout_commands', Pose2D, command_process)
        scout_pub = threading.Thread(target=scout_publisher,args=(motion_pub,))
        time.sleep(2)
        scout_pub.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
