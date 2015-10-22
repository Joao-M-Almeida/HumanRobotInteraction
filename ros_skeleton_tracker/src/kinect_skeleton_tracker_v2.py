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
        '/head',
        '/neck',
        '/torso',
        '/left_shoulder',
        '/left_elbow',
        '/left_hand',
        '/left_hip',
        '/left_knee',
        '/left_foot',
        '/right_shoulder',
        '/right_elbow',
        '/right_hand',
        '/right_hip',
        '/right_knee',
        '/right_foot'
        ]
Coord = [
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0],
		[0.0,0.0,0.0]
		]
        
if __name__ == '__main__':
    rospy.init_node('kinect_tracking')

    listener = tf.TransformListener()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
    	
    	for master in range(1,17):
			st='/head_' + str(master)
			if(listener.waitForTransform(BASE_FRAME, st, rospy.Time(), rospy.Duration(0.25))):
				break
			else:
				continue
			#try:
				#print(listener.lookupTransform(BASE_FRAME, st, rospy.Time(0)))
				#break
			#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
   				#continue
   		
    	if master == 16:
			print('No master found')
			continue
    	print(master)		
    	
    	try:
        	for f in range(0,len(FRAMES)):
				st='_' + str(master)
				coordinates = listener.lookupTransform(BASE_FRAME, FRAMES[f] + st, rospy.Time(0))
				Coord[f] = coordinates[0]
            	
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        	continue
        
        print(Coord)
        
		#print("Head_:" + trans + rot)
        
        rate.sleep()
