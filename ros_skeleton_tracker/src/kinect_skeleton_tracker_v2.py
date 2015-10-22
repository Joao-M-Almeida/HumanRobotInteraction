#!/usr/bin/env python

"""Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.
"""


import roslib
import rospy
import tf
import math
import geometry_msgs.msg

BASE_FRAME = '/openni_depth_frame'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]
        
if __name__ == '__main__':
    rospy.init_node('kinect_tracking')

    listener = tf.TransformListener()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
    	try:
        	#(trans,rot) = listener.lookupTransform('camera_link', '/camera_rgb_frame', rospy.Time(0))
            #(trans,rot) = listener.lookupTransform('/openni_depth_frame', '/head_3', rospy.Time(0))
            print(listener.lookupTransform('/openni_depth_frame', '/head_1', rospy.Time(0)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        	continue
        
		#print("Head_:" + trans + rot)
        
        rate.sleep()
