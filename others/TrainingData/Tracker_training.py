#!/usr/bin/env python

#Module to connect to a kinect through ROS + OpenNI and access
#the skeleton postures.
#


import roslib
import rospy
import tf
import math
import geometry_msgs.msg
import time

#debug stuff
import curses
import atexit
import numpy
#import print_function

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
Coord_3D=[[[0.0 for k in xrange(3)] for j in xrange(15)] for i in xrange(30)]

vector_head = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
vector_hand = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
vector_shou = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

Database = [0.0, 0.0, 0.0, 0.0]
stdscr = curses.initscr()
file_out=file

def exit_handler():
    curses.endwin()
    file_out.close()
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
	
	file_out = open('Database.txt', 'a')
	
	rospy.init_node('kinect_tracking')

	listener = tf.TransformListener()

	rate = rospy.Rate(10)
	t=0
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
		        Coord_3D[t][f] = coordinates[0]
		        stdscr.addstr(16, 0, str(t) + '\t')
		    #outside de for#
		    t = t+1
		    if t >= 30:
		    	t = 0

		        for i in range(0,30):
		            if i== 28:
		                vector_head[i] = Coord_3D[i][0][0] - Coord_3D[i+1][0][0] #Gradiente cabeca
		            else:
		                vector_head[i] = Coord_3D[i-1][0][0] - Coord_3D[i][0][0]

		            vector_hand[i] = Coord_3D[i][5][2] - Coord_3D[i][11][2]  #Diferenca entre maos#
		            vector_shou[i] = Coord_3D[i][3][2] - Coord_3D[i][9][2]   #Diferenca entre ombros#
		        
		        if(numpy.mean(Coord_3D[0:30][5][2]) > numpy.mean(Coord_3D[0:30][0][2])):
		            higher = 1 #-> Hands higher than head"
		        else:
		            higher = 0 #-> Hands higher lower head"

		        Database[0] = numpy.mean(vector_head) #Media X Gradiente da cabeca#
		        Database[1] = numpy.mean(vector_hand) #Media Z Altura das maos #
		        Database[2] = numpy.mean(vector_shou) #Media Z Altura dos ombros#
		        Database[3] = higher                  #Maos  Z acima da cabeca#

		        file_out.write(str(Database) + '\n')
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		    continue

		print_coord()

		#print("Head_:" + trans + rot)

		rate.sleep()
