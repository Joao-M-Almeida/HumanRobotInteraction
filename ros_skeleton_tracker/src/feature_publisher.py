#!/usr/bin/env python

'''
    Feature Node
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
vx=0
vy=0
x=0.0
y=0.0
ngest  = 5
data_base=[[0.0 for k in xrange(ngest*4 + 2)]]

gesture_dict = {
                '0.0': 0,
                'closing forearm': 1,
                'arm moving down': 2,
                'arm moving up': 3,
                'arm stopped': 4,
                'forearm stopped': 5,
                'hand above elbow': 6,
                'hand under elbow': 7,
                'opening forearm': 8,
                'walking backward': 9,
                'walking forward': 10,
                'waving': 11,
                'no walking': 12,
                'calling': 13,
                'handing': 14,
                'elbow behind body': 15,
                'elbow in front of body': 16
                }


class feature_publisher:
    pub = rospy.Publisher('/features', String, queue_size=10)

    @staticmethod
    def publish(message):
        feature_publisher.pub.publish(message)

def new_position(pose):
    global x
    global y
    global vx
    global vy
    global x_past
    global y_past

    x=pose.x
    y=pose.y
    vx=x-x_past
    vy=y-y_past

def new_gesture(gesture):
    global x
    global y
    global vx
    global vy

    build_database(gesture, x, y, vx, vy)

# Order or the vector inputs: Gesture(n) | delta_t(n) | Speed_x(n) | Gesture(n-1) | delta_t(n-1) | Speed_x(n-1) | ... Distance(n)
def build_database(gesture,pose_x,pose_y, velx, vely):
    global x_past
    global y_past
    global data_base
    feature_vector=[0.0 for k in xrange(ngest*4 + 2)]

    # Computacao do gesto actual
    msecs        = gesture.header.stamp.nsecs/1000000
    gesture_time = gesture.header.stamp.secs + (0.001*msecs)

    D_x    = pose_x
    D_y    = pose_y

    # Assembly feature vector
    feature_vector[0] = D_x
    feature_vector[1] = D_y
    feature_vector[2] = gesture.gesture
    feature_vector[3] = gesture_time
    feature_vector[4] = velx
    feature_vector[5] = vely

    vector_aux = data_base[len(data_base)-1]
    feature_vector[6:ngest*4 + 2] = vector_aux[2:(ngest-1)*4 + 2]

    feature_vector[7]=feature_vector[3]-feature_vector[7]
    feature_vector[11]=feature_vector[11]-feature_vector[7]
    feature_vector[15]=feature_vector[15]-feature_vector[7]
    feature_vector[19]=feature_vector[19]-feature_vector[7]

    #publish to topic
    print feature_vector[2]
    for key, value in gesture_dict.iteritems():
        if(feature_vector[2] == key):
            feature_vector[2] = value
            break
    else:
        feature_vector[2] = 0

    print feature_vector[2]

    feature_string = str(feature_vector)

    feature_string = feature_string.replace(" ", "")
    feature_string = feature_string.replace("[", "")
    feature_string = feature_string.replace("]", "")

    feature_publisher.publish(feature_string)

    #store in data_base
    data_base.append(feature_vector)

    x_past = D_x
    y_past = D_y

def goodbye():
    rospy.loginfo('Exiting...')

if __name__ == '__main__':
    atexit.register(goodbye)
    rospy.init_node('hri_decision_node', anonymous=True)

    rate = rospy.Rate(10)
    rospy.Subscriber('/gestures', gesture, new_gesture)
    rospy.Subscriber('/masterlocation', pose_msg, new_position)

    while not rospy.is_shutdown():
        rate.sleep()
