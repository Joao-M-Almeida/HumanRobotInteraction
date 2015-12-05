#!/usr/bin/env python

import rospy
from ros_skeleton_tracker.msg import gesture
from ros_skeleton_tracker.msg import pose_msg
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import threading
import atexit

'''
    wave = 0
    low_five = 1
    high_five = 2
    clap = 3
    '''
alive=True
n_cmds = 4
#prob_cmd = {}
threshold = 0.8
default_prob = 1.0/n_cmds

master_position = [(0,0), (0, 0), (0, 0), (0, 0), (0, 0)]

commands = {'wave': default_prob, 'low_five': default_prob, 'high_five': default_prob, 'clap': default_prob}

kat_pub = rospy.Publisher('/katana_commands', String, queue_size=10)
scout_pub = rospy.Publisher('/scout_commands', Pose2D, queue_size=10)

kinect_info = []

def new_gesture(gesture):
    msg = String()
    msg.data = 'wave'
    if gesture.gesture == 'waving':
        kat_pub.publish(msg)
        rospy.loginfo("Sending to Katana"+str(msg))
    kinect_info.append((gesture.gesture,gesture.header.stamp.secs))

def new_position(pose):
    master_position.pop(0)
    master_position.append((pose.x,pose.y))

def normallize_cmd_prob():
    prob_sum=0
    for cmd, prob in commands.iteritems():
        prob_sum=prob_sum+prob
    for cmd in commands:
        commands[cmd]=prob/prob_sum

def action_controller():
    global alive
    global done
    rate = rospy.Rate(0.5)
    while(True):
        #rospy.loginfo("Kinect gestures: " + str(kinect_info))
        #rospy.loginfo("Master: " + str(master_position))
        for cmd, prob in commands.iteritems():
            if prob > threshold and done==1:
                rospy.loginfo("Executing: " + str(cmd))
                commands[cmd] = default_prob
                normallize_cmd_prob()
                done == 0

        rate.sleep()

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

def feedback(str_a):
    global done
    done = 1


def new_movement(data):
    moveit = data.movement
    msg.data = ''
    #Katana movements
    if moveit.lower()=='waving':
        msg.data = 'wave'

    elif moveit.lower()=='grabbing':
        msg.data = 'grab'

    elif moveit.lower()=='high_five':
        msg.data = 'high_five'

    elif moveit.lower()=='low_five':
        msg.data = 'low_five'

    if msg.data != '':
        kat_pub.publish(msg)
        rospy.loginfo("Sending to Katana"+str(msg))
        kinect_info.append((gesture.gesture,gesture.header.stamp.secs))


    msg.x = ''
    if moveit.lower()=='going':
        msg.x=master_position[4][0]-scout_x-0.5     #0.5: zona pessoal
    elif moveit.lower()=='going_back':
        msg.x=-scout_x

    if msg.x != '':
        msg.y = 0
        msg.theta = 0
        #publisher do scout
        scout_pub.publish(msg)
        rospy.loginfo("Sending to Scout"+str(msg))


def goodbye():
    rospy.loginfo('Exiting...')

if __name__ == '__main__':
    atexit.register(goodbye)

    try:
        rospy.init_node('hri_decision_node', anonymous=True)
        #rospy.Subscriber('/gestures', gesture, new_gesture)
        rospy.Subscriber('/masterlocation', pose_msg, new_position)
        rospy.Subscriber('/movements', String, new_movement)
        rospy.Subscriber('/odom', Odometry, odom_process)
        rospy.Subscriber('/actionfeedback', String, feedback)

        rate = rospy.Rate(0.5)

        ##while(True):
        ##    print var
        rate.sleep()

        rospy.loginfo("Commands :" + str(commands))

        check_command = threading.Thread(target=action_controller)
        check_command.setDaemon(True)
        check_command.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
