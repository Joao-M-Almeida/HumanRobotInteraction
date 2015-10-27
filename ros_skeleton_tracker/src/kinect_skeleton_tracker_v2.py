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
alfa = 0.0
beta = 0.0

stdscr = curses.initscr()



def exit_handler():
    curses.endwin()
    print 'Exiting!'

def print_coord():
	for f in range(0,len(FRAMES)):
		string=str(FRAMES[f]) + '\t->\t' + str(['%.6f' % elem for elem in Coord[f]]) + '\t'
		stdscr.addstr(f, 0, string)
		#print FRAMES[f] , '->', Coord[f]
	
	angle_string='Angle alfa\t->\t' + str(alfa)
	stdscr.addstr(len(FRAMES), 0, angle_string)
	
	angle_string='Angle beta\t->\t' + str(beta)
	stdscr.addstr(len(FRAMES)+1, 0, angle_string)
	
	stdscr.refresh()
	
def dotproduct(v1, v2):
	return sum((a*b) for a, b in zip(v1, v2))

def length(v):
	return math.sqrt(dotproduct(v, v))

def angles():
	global alfa, beta, gama
	lsh_elb=[Coord[3][0]-Coord[4][0], Coord[3][1]-Coord[4][1], Coord[3][2]-Coord[4][2]]
	lhan_elb=[Coord[5][0]-Coord[4][0], Coord[5][1]-Coord[4][1], Coord[5][2]-Coord[4][2]]
	alfa=math.degrees(math.acos(dotproduct(lsh_elb, lhan_elb) / (length(lsh_elb) * length(lhan_elb))))

	torso_neck=[Coord[2][0]-Coord[1][0], Coord[2][1]-Coord[1][1], Coord[2][2]-Coord[1][2]]
	elb_lsh=[Coord[4][0]-Coord[3][0], Coord[4][1]-Coord[3][1], Coord[4][2]-Coord[3][2]]
	beta=math.degrees(math.acos(dotproduct(torso_neck, elb_lsh) / (length(torso_neck) * length(elb_lsh))))
	
	torso_neck=[Coord[2][0]-Coord[1][0], Coord[2][1]-Coord[1][1], Coord[2][2]-Coord[1][2]]
	sholder_neck=[Coord[3][0]-Coord[1][0], Coord[3][1]-Coord[1][1], Coord[3][2]-Coord[1][2]]
	
	
	
	        
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
		
		angles()
		#left_angle=1.0
		print_coord()

		#print("Head_:" + trans + rot)

		rate.sleep()
