#!/usr/bin/env python

import rospy
from ros_skeleton_tracker.msg import gesture
from ros_skeleton_tracker.msg import pose_msg
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import threading
import atexit
import pandas as pd
from sklearn.svm import SVC
from sklearn.decomposition import PCA
import numpy as np
import time

'''
    nop = 1
    wave = 2
    h5 = 3
    l5 = 4
    go = 5
    grab = 6
    go_back = 7
'''

alive = True
n_cmds = 7
threshold = 0.4
default_prob = 1.0/n_cmds
done = 1
increment = 0.3

last_cmd = -1

master_position = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]

commands = {1: default_prob, 2: default_prob, 3: default_prob, 4: default_prob, 5: default_prob, 6: default_prob, 7: default_prob}

kat_pub = rospy.Publisher('/katana_commands', String, queue_size=10)
scout_pub = rospy.Publisher('/scout_commands', Pose2D, queue_size=10)

kinect_info = []

pca = PCA(n_components = 15)
classf = SVC()

def do_nop():
    rospy.loginfo("             Executing: NOP")
    global done
    time.sleep(1)
    rospy.loginfo("Executed")
    done = 1
    for cmd1, prob1 in commands.iteritems():
        commands[cmd1] = default_prob

def do_wave():
    rospy.loginfo("             Executing: WAVE")
    global done
    kat_pub.publish('wave')

def do_l5():
    rospy.loginfo("             Executing: L5")
    global done
    kat_pub.publish('low_five')


def do_h5():
    rospy.loginfo("             Executing: H5")
    global done
    kat_pub.publish('high_five')

def do_grab():
    rospy.loginfo("             Executing: GRAB")
    global done
    kat_pub.publish('grabbing')

def do_go():
    rospy.loginfo("             Executing: GO")
    global done
    global scout_x
    global master_position

    msg = Pose2D()
    msg.x=master_position[4][0]-scout_x-0.5     #0.5: zona pessoal

    scout_pub.publish(msg);
    '''time.sleep(1)
    rospy.loginfo("Executed")
    done = 1
    for cmd1, prob1 in commands.iteritems():
        commands[cmd1] = default_prob'''

def do_go_back():
    rospy.loginfo("             Executing: GO BACK")
    global done
    global scout_x

    msg = Pose2D()
    msg.x=-scout_x

    scout_pub.publish(msg);


def train_classifier():

    global pca
    global classf

    database = pd.read_csv( "/home/filipe/Documents/SisAut/HumanRobotInteraction/others/TrainingData/TUDO2.csv", quotechar ="'")

    labels = database['label'].values
    database.drop("label",axis=1,inplace=True)
    features = database.values

    pca.fit(features)
    features = pca.transform(features)

    classf.fit(features,labels)

'''
def new_gesture(gesture):
    msg = String()
    msg.data = 'wave'
    if gesture.gesture == 'waving':
        kat_pub.publish(msg)
        rospy.loginfo("Sending to Katana"+str(msg))
    kinect_info.append((gesture.gesture,gesture.header.stamp.secs))
'''

'''
At each new pose msg, update internal master position
'''
def new_position(pose):
    master_position.pop(0)
    master_position.append((pose.x,pose.y))

def normallize_cmd_prob():
    prob_sum=0
    for cmd, prob in commands.iteritems():
        prob_sum=prob_sum+prob
    #rospy.loginfo("Prob sum = " + str(prob_sum))
    for cmd, prob in commands.iteritems():
        commands[cmd]=prob/prob_sum

def action_controller():
    global alive
    global done
    global last_cmd
    rate = rospy.Rate(5)
    while(True):
        #rospy.loginfo("Kinect gestures: " + str(kinect_info))
        #rospy.loginfo("Master: " + str(master_position))
        for cmd, prob in commands.iteritems():
            if prob > threshold:
                if done == 1 and last_cmd != cmd:
                    done = 0
                    if cmd == 1:
                        do_nop()
                    elif cmd == 2:
                        do_wave()
                    elif cmd == 3:
                        do_h5()
                    elif cmd == 4:
                        do_l5()
                    elif cmd == 5:
                        do_go()
                    elif cmd == 6:
                        do_grab()
                    elif cmd == 7:
                        do_go_back()
                    last_cmd = cmd
                    for cmd1, prob1 in commands.iteritems():
                        commands[cmd1] = default_prob
                else:
                    for cmd1, prob1 in commands.iteritems():
                        commands[cmd1] = default_prob
        rate.sleep()

'''
At each new odom msg, update internal scout position and rotation
'''
def odom_process(data):
    quaternion=[0, 0, 0, 0]
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
    rospy.loginfo("Executed")
    for cmd1, prob1 in commands.iteritems():
        commands[cmd1] = default_prob


'''
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
'''

'''
At each new feature vector from the kinect predict which action the robot should
do, use that to update the probabilities for each command.
'''
def new_feature(feature_str):
    feature = np.fromstring(feature_str.data,dtype = float , sep = ',')
    new_sample = pca.transform(feature)
    predicted_action = classf.predict(new_sample)
    #rospy.loginfo("Received : \n" + feature_str.data + "\n Predicted: " + str(predicted_action))
    rospy.loginfo("Predicted: " + str(predicted_action))
    #if predicted_action != 4:
    for cmd, prob in commands.iteritems():
        if cmd == predicted_action:
            #rospy.loginfo("Incrementing command: " + str(cmd))
            commands[cmd] = prob + increment
    #rospy.loginfo("Commands :" + str(commands))
    normallize_cmd_prob()
    #rospy.loginfo("Commands :" + str(commands))


def goodbye():
    rospy.loginfo('Exiting...')

if __name__ == '__main__':
    atexit.register(goodbye)

    try:

        train_classifier()

        rospy.init_node('hri_decision_node', anonymous=True)

        rospy.Subscriber('/masterlocation', pose_msg, new_position)
        rospy.Subscriber('/odom', Odometry, odom_process)

        rospy.Subscriber('/features', String, new_feature)

        rospy.Subscriber('/actionfeedback', String, feedback)

        time.sleep(0.5)

        rospy.loginfo("Commands :" + str(commands))

        check_command = threading.Thread(target=action_controller)
        check_command.setDaemon(True)
        check_command.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
