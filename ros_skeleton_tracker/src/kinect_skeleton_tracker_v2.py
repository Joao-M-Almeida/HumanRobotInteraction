#!/usr/bin/env python

"""Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.
"""


import roslib
import rospy
import tf
import math
import geometry_msgs.msg
import time
import numpy as np
#debug stuff
import curses
import atexit


BASE_FRAME = '/openni_depth_frame'
FRAME_COUNT = 15
MEDIANSIZE = 5
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

Coordint = [
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0],
        [0,0,0]
        ]

#arm angles
alfa = [0.0,0.0]
beta = [0.0,0.0]
gama = [0.0,0.0]
sigma =[0.0,0.0]

#arm angular velocities
valfa= [0.0,0.0]
vbeta= [0.0,0.0]
vgama= [0.0,0.0]
vsigma=[0.0,0.0]

#arm angular acceleration
aalfa=0.0
abeta=0.0
agama=0.0
asigma=0.0
state_forearm=' '
state_arm=' '
state_arm2=' '
stat=1
#state=0
file_out=file
flag=0
flag1=0
flag2=0
stdscr = curses.initscr()

position=[0.0,0.0]
vel=[0.0,0.0]
acel=0.0


def exit_handler():
    curses.endwin()
    file_out.close()
    print 'Exiting!'

def print_coord():
    for f in range(0,len(FRAMES)):
        string=str(FRAMES[f]) + '\t->\t' + str(['%0.3f' % elem for elem in Coord[f]]) + '\t'

        stdscr.addstr(f, 0, string)
        #print FRAMES[f] , '->', Coord[f]

    angle_string='Angle alfa\t->\t' + str(alfa) + '\t\t'
    stdscr.addstr(len(FRAMES), 0, angle_string)

    angle_string='Angle beta\t->\t' + str(beta) + '\t\t'
    stdscr.addstr(len(FRAMES)+1, 0, angle_string)

    angle_string='Angle gama\t->\t' + str(gama) + '\t\t'
    stdscr.addstr(len(FRAMES)+2, 0, angle_string)

    angle_string='Angle valfa\t->\t' + str(valfa) + '\t\t'
    stdscr.addstr(len(FRAMES)+3, 0, angle_string)

    angle_string='Angle vbeta\t->\t' + str(vbeta) + '\t\t'
    stdscr.addstr(len(FRAMES)+4, 0, angle_string)

    angle_string='Angle vgama\t->\t' + str(vgama) + '\t\t'
    stdscr.addstr(len(FRAMES)+5, 0, angle_string)



    stdscr.refresh()

def dotproduct(v1, v2):
    return sum((a*b) for a, b in zip(v1, v2))

def length(v):
    return math.sqrt(dotproduct(v, v))

def angles():
    global alfa, beta, gama
    alfa[1]=alfa[0]
    beta[1]=beta[0]
    gama[1]=gama[0]

    valfa[1]=valfa[0]
    vbeta[1]=vbeta[0]
    vgama[1]=vgama[0]

    ##forearm##
    #angle between arm and forearm
    lsh_elb=[Coord[3][0]-Coord[4][0], Coord[3][1]-Coord[4][1], Coord[3][2]-Coord[4][2]]
    lhan_elb=[Coord[5][0]-Coord[4][0], Coord[5][1]-Coord[4][1], Coord[5][2]-Coord[4][2]]
    if(length(lsh_elb)!=0.0 and length(lhan_elb)!=0.0):
        alfa[0]=math.degrees(math.acos(dotproduct(lsh_elb, lhan_elb) / (length(lsh_elb) * length(lhan_elb))))

    alfa[0]=int(alfa[0])

    ##arm##
    #angle between arm and spine
    torso_neck=[Coord[2][0]-Coord[1][0], Coord[2][1]-Coord[1][1], Coord[2][2]-Coord[1][2]]
    elb_lsh=[Coord[4][0]-Coord[3][0], Coord[4][1]-Coord[3][1], Coord[4][2]-Coord[3][2]]
    if(length(torso_neck)!=0.0 and length(elb_lsh)!=0.0):
        beta[0]=math.degrees(math.acos(dotproduct(torso_neck, elb_lsh) / (length(torso_neck) * length(elb_lsh))))

    beta[0]=int(beta[0])
    #angle between arm and chest
    sholderr_sholderl=[Coord[9][0]-Coord[3][0], Coord[9][1]-Coord[3][1], Coord[9][2]-Coord[3][2]]
    elb_shl=[Coord[4][0]-Coord[3][0], Coord[4][1]-Coord[3][1], Coord[4][2]-Coord[3][2]]
    if(length(sholderr_sholderl)!=0.0 and length(elb_shl)!=0.0):
        gama[0]=math.degrees(math.acos(dotproduct(sholderr_sholderl, elb_shl) / (length(sholderr_sholderl) * length(elb_shl))))

    gama[0]=int(gama[0])
    #angle between

    valfa[0]=alfa[0]-alfa[1]
    vbeta[0]=beta[0]-beta[1]
    vgama[0]=gama[0]-gama[1]

    aalfa=valfa[0]-valfa[1]
    abeta=vbeta[0]-vbeta[1]
    agama=vgama[0]-vgama[1]

    #disthand=[Coord[4][0]-Coord[3][0], Coord[4][1]-Coord[3][1], Coord[4][2]-Coord[3][2]]

def waving():
    angle_string='\t\t'
    global flag_left
    global flag_rigth
    if beta[0] > 70 and beta[0]< 120:
        if valfa[0]<0:
            flag_rigth = 1
            if flag_left == 1:
                angle_string='waving'
            flag_left = 0

        if valfa[0] > 0:
            flag_left = 1
            if flag_rigth == 1:
                angle_string='waving'
            flag_rigth = 0
    else:
        flag_left=0
        flag_rigth=0

    stdscr.addstr(len(FRAMES)+12, 0, angle_string)

def walk():
    position[1]=position[0]
    position[0]=(Coord[2][0]+ Coord[6][0]+ Coord[12][0]+ Coord[1][0])/4

    vel[1]=vel[0]
    vel[0]=position[0]-position[1]

    acel=vel[0]-vel[1]

    p=str(position)

    if vel[0]>0.015:
        stdscr.addstr(len(FRAMES)+7, 0, 'walking backward')
    elif vel[0]<-0.015:
        stdscr.addstr(len(FRAMES)+7, 0, 'walking forward')
    else:
        stdscr.addstr(len(FRAMES)+7, 0, 'stopped\t\t')

	stdscr.addstr(len(FRAMES)+8, 0, p)

def hand():
	if vbeta[0] > 5:
		stdscr.addstr(len(FRAMES)+9, 0, 'arm moving up\t\t')
	elif vbeta[0] < 5:
		stdscr.addstr(len(FRAMES)+9, 0, 'arm moving down\t\t')
	if valfa[0] > 5:
		stdscr.addstr(len(FRAMES)+10, 0, 'openning arm\t\t')
	elif valfa[0] < 5:
		stdscr.addstr(len(FRAMES)+10, 0, 'closing arm\t\t')

	if Coord[5][2] > Coord[4][2]:
		stdscr.addstr(len(FRAMES)+11, 0, 'Hand above elbow')
	elif Coord[5][2] < Coord[4][2]:
		stdscr.addstr(len(FRAMES)+11, 0, 'Hand under elbow')


if __name__ == '__main__':
    stdscr = curses.initscr()
    print_coord()
    atexit.register(exit_handler)
    rospy.init_node('kinect_tracking')

    file_out = open('Database.txt', 'a')
    time_coord=[[[0,0,0] for t in range(MEDIANSIZE)] for i in range(FRAME_COUNT)]

    listener = tf.TransformListener()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #for master in range(1,17):
        #   st='/torso_' + str(master)

        #   try:
            #   found=listener.waitForTransform(BASE_FRAME, st, rospy.Time(), rospy.Duration(0.2))
            #except tf.Exception as e:
            #   print "some tf exception happened", e.args
            #   continue

            #if found:
            #   break
            #else:
            #   continue
            #try:
            #   print(listener.lookupTransform(BASE_FRAME, st, rospy.Time(0)))
            #   break
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #   continue

        #if master == 16:
        #   print('No master found')
        #   continue

        master=2

        try:
            for f in range(0,len(FRAMES)):
                st='_' + str(master)
                coordinates = listener.lookupTransform(BASE_FRAME, FRAMES[f] + st, rospy.Time(0))
                #delete oldest & append to 3d array
                time_coord[f].pop(0)
                time_coord[f].append(list(coordinates[0]))
                current = time_coord[f]
                #print(current)
                #apply median filter
                median_res = [np.median([current[i][j] for i in range(MEDIANSIZE)]) for j in range(3)]
                #time.sleep(30)
                #coordinates[0] = filtered results
                Coord[f] = median_res
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angles()

        print_coord()
        waving()
        walk()
        hand()
        #forearm()
        #arm()
        #state()


        rate.sleep()
