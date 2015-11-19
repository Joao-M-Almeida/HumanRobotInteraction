#!/usr/bin/env python


'''
    learning Node
'''

import rospy
from ros_skeleton_tracker.msg import gesture
from ros_skeleton_tracker.msg import pose_msg
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import threading
import atexit

x_past = 0.0
y_past = 0.0
x=0.0
y=0.0
ngest  = 5
data_base=[[0.0 for k in xrange(ngest*4 + 2)]]



'''
    wave        = 0
    low_five    = 1
    high_five   = 2
    clap        = 3
    call        = 4
    grasp       = 5
    nop         = 6
    '''

def new_position(pose):
    global x
    global y

    x=pose.x
    y=pose.y

def new_gesture(gesture):
    global x
    global y

    build_database(gesture, x, y)

# Order or the vector inputs: Gesture(n) | delta_t(n) | Speed_x(n) | Gesture(n-1) | delta_t(n-1) | Speed_x(n-1) | ... Distance(n)
def build_database(gesture,pose_x,pose_y):
    global x_past
    global y_past
    global data_base
    feature_vector=[0.0 for k in xrange(ngest*4 + 2)]

    # Computacao do gesto actual
    msecs        = gesture.header.stamp.nsecs/1000000
    gesture_time = gesture.header.stamp.secs + (0.001*msecs)

    D_x    = pose_x
    D_y    = pose_y

    velx   = D_x - x_past
    vely   = D_y - y_past

    x_past = D_x
    y_past = D_y


    # Assemble do feature vector
    feature_vector[0] = D_x
    feature_vector[1] = D_y
    feature_vector[2] = gesture.gesture
    feature_vector[3] = gesture_time
    feature_vector[4] = velx
    feature_vector[5] = vely

    vector_aux = data_base[len(data_base)-1]
    feature_vector[6:ngest*4 + 2] = vector_aux[2:(ngest-1)*4 + 2]

    print str(feature_vector)


    #save to file
    file_out = open('build_database.txt', 'a')
    file_out.write(str(feature_vector) + '\n')
    file_out.close()

    data_base.append(feature_vector)

    #data_base.append((D_x, D_y, gesture.gesture, gesture_time, velx, vely, feature_vector[0:(number_gestures-1)*3]))

def goodbye():
    rospy.loginfo('Exiting...')

if __name__ == '__main__':
    atexit.register(goodbye)
    rospy.init_node('hri_decision_node', anonymous=True)

    rate = rospy.Rate(0.5)
    rospy.Subscriber('/gestures', gesture, new_gesture)
    rospy.Subscriber('/masterlocation', pose_msg, new_position)

    while not rospy.is_shutdown():
        rate.sleep()
