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
#prob_cmd = {}
threshold = 0.8
default_prob = 1.0/n_cmds
done = 1

master_position = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0)]

commands = {1: default_prob, 2: default_prob, 3: default_prob, 4: default_prob, 5: default_prob, 6: default_prob, 7: default_prob}

kat_pub = rospy.Publisher('/katana_commands', String, queue_size=10)
scout_pub = rospy.Publisher('/scout_commands', Pose2D, queue_size=10)

kinect_info = []

pca = PCA(n_components = 15)
classf = SVC()

def train_classifier():

    global pca
    global classf

    database = pd.read_csv( "/home/jmirandadealme/Documents/SistAut/HumanRobotInteraction/others/TrainingData/todos2.txt", quotechar ="'")

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
            if prob > threshold and done == 1:
                rospy.loginfo("Executing: " + str(cmd))
                commands[cmd] = default_prob
                normallize_cmd_prob()
                done == 0

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
    predicted_action = classf.predict(sample)
    rospy.loginfo("Received : \n" + feature_str.data + "\n Predicted: " + str(predicted_action))
    for cmd, prob in commands.iteritems():
        if cmd == predicted_action:
            commands[cmd] = prob + increment
    normallize_cmd_prob()



def goodbye():
    rospy.loginfo('Exiting...')

if __name__ == '__main__':
    atexit.register(goodbye)

    try:

        train_classifier()

        rospy.init_node('hri_decision_node', anonymous=True)

        rospy.Subscriber('/masterlocation', pose_msg, new_position)
        rospy.Subscriber('/odom', Odometry, odom_process)

#        rospy.Subscriber('/movements', String, new_movement)

        rospy.Subscriber('/feature_vector', String, new_feature)

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
