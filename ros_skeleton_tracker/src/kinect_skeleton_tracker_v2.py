#!/usr/bin/env python

"""Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.
"""


import roslib
import rospy
import tf
import math
import geometry_msgs.msg

#debug stuff
import curses
import atexit


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

stdscr = curses.initscr()


def exit_handler():
    curses.endwin()
    print 'Exiting!'

def print_coord():
	for f in range(0,len(FRAMES)):
		string=str(FRAMES[f]) + '\t->\t' + str(['%.6f' % elem for elem in Coord[f]]) + '\t'
		stdscr.addstr(f, 0, string)
		#print FRAMES[f] , '->', Coord[f]
	
	stdscr.refresh()
        
if __name__ == '__main__':
	stdscr = curses.initscr()
	print_coord()
	atexit.register(exit_handler)
	rospy.init_node('kinect_tracking')

	listener = tf.TransformListener()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#for master in range(1,17):
			#st='/torso_' + str(master)

			#try:
				#found=listener.waitForTransform(BASE_FRAME, st, rospy.Time(), rospy.Duration(0.2))
			#except tf.Exception as e:
				#print "some tf exception happened", e.args
				#continue

			#if found:
				#break
			#else:
				#continue
			#try:
				#print(listener.lookupTransform(BASE_FRAME, st, rospy.Time(0)))
				#break
			#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				#continue

		#if master == 16:
			#print('No master found')
			#continue
		
		master=1
		#print(master)		

		try:
			for f in range(0,len(FRAMES)):
				st='_' + str(master)
				coordinates = listener.lookupTransform(BASE_FRAME, FRAMES[f] + st, rospy.Time(0))
				Coord[f] = coordinates[0]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		print_coord()

		#print("Head_:" + trans + rot)

		rate.sleep()
