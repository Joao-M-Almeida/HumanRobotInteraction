#!/usr/bin/env python

"""Module to connect to a kinect through ROS + OpenNI and access
the skeleton postures.
"""


import roslib
import rospy
import tf
import math
import geometry_msgs.msg
import std_msgs.msg
from ros_skeleton_tracker.msg import pose_msg
from ros_skeleton_tracker.msg import gesture
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
flag_left=0
flag_rigth=0
flag_f=0
flag_b=0
stdscr = curses.initscr()

position=[0.0,0.0,0.0,0.0]
vel=[0.0,0.0]
acel=0.0
wave_string=' '
walk_string=' '
call_string=' '
string_arm=' '
string_arm1=' '
string_walk=' '
string_hand=' '

class gesture_publisher:
    pub = rospy.Publisher('/gestures', gesture, queue_size=10)
    gestures = gesture()

    #def __init__(self):
        #pub = rospy.Publisher('/gestures', pose_msg, queue_size=10)
        #gestures = pose_msg()
        #self.gestures.header.stamp = rospy.Time.now()
        #self.gestures.gesture='ahoy!'
        #self.pub.publish(self.gestures)

    def publish(self, message):
        self.gestures.header.stamp = rospy.Time.now()
        self.gestures.gesture=message
        self.pub.publish(self.gestures)

class masterlocation_publisher:
    pub = rospy.Publisher('/masterlocation', pose_msg, queue_size=10)
    masterlocation = pose_msg()

    def __init__(self):
        self.masterlocation.header.stamp = rospy.Time.now()
        self.masterlocation.id=0
        self.masterlocation.x=0
        self.masterlocation.y=0
        self.pub.publish(self.masterlocation)

    def publish(self, masterid, x, y=0):
        self.masterlocation.header.stamp = rospy.Time.now()
        self.masterlocation.id=masterid
        self.masterlocation.x=x
        self.masterlocation.y=y
        self.pub.publish(self.masterlocation)

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
    global valfa, vbeta, vgama
    global aalfa, abeta, agama
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

    valfa[0]=alfa[0]-alfa[1]
    vbeta[0]=beta[0]-beta[1]
    vgama[0]=gama[0]-gama[1]

    aalfa=valfa[0]-valfa[1]
    abeta=vbeta[0]-vbeta[1]
    agama=vgama[0]-vgama[1]

    #disthand=[Coord[4][0]-Coord[3][0], Coord[4][1]-Coord[3][1], Coord[4][2]-Coord[3][2]]

def waving(publ):
    temp = ''
    global wave_string
    global flag_left
    global flag_rigth
    if beta[0] > 60 and beta[0]< 120 and gama[0] > 150:
        if valfa[0] < -5:
            flag_rigth = 1
            if flag_left == 1:
                temp='waving'
            #else:
                #temp='no wave'
            flag_left = 0
        elif valfa[0] > 5:
            flag_left = 1
            if flag_rigth == 1:
                temp='waving'
            #else:
                #temp='no wave'
            flag_rigth = 0
        #else:
            #temp=''
            #temp='no wave'
    else:
        flag_left=0
        flag_rigth=0
        temp='no wave'

    if wave_string!=temp and temp!='':
        wave_string=temp
        #stdscr.addstr(len(FRAMES)+:12, 0, wave_string + '\t\t')
        if wave_string!='no wave':
            publ.publish(wave_string)

def calling(publ):
    temp = ''
    global call_string
    global flag_f
    global flag_b
    if beta[0] > 60 and beta[0]< 120 and gama[0] > 60 and gama[0] < 140:
        if valfa[0] < -5:
            flag_b = 1
            if flag_f == 1:
                temp='calling'
            #else:
                #temp='no wave'
            flag_f = 0
        elif valfa[0] > 5:
            flag_f = 1
            if flag_b == 1:
                temp='calling'
            #else:
                #temp='no wave'
            flag_b = 0
        #else:
            #temp=''
            #temp='no wave'
    else:
        flag_f=0
        flag_b=0
        temp='no calling'

    if call_string!=temp and temp!='':
        call_string=temp
        #stdscr.addstr(len(FRAMES)+:12, 0, wave_string + '\t\t')
        if call_string!='no calling':
            publ.publish(call_string)

def walk(publ):
    #avaliar possibilidade de usar 2 coordenadas para a position
    temp=''
    global walk_string
    global position
    #position[1:4]=position[0:3]
    '''
    1  - /neck
    2  - /torso
    6  - /left_hip
    12 - /right_hip
    '''
    #position[0]=(Coord[2][0]+ Coord[6][0]+ Coord[12][0]+ Coord[1][0])/4

    #vel[1]=vel[0]
    vel[0]=position[0]-position[3]

    #acel=vel[0]-vel[1]

    if abs(vel[0])>0.1:
        temp = 'walking backward'
        if vel[0]<0:
            temp = 'walking forward'
    else:
        temp = 'no walking'

    if walk_string!=temp:
        walk_string=temp
        stdscr.addstr(len(FRAMES)+7, 0, walk_string+'\t\t')
        publ.publish(walk_string)

def hand(publ):
    global string_arm
    global string_arm1
    global string_hand
    temp=''
    if abs(vbeta[0]) > 10:
        temp='arm moving up'
        if vbeta[0] < 0:
            temp='arm moving down'
    else:
        temp='arm stopped'

    if string_arm!=temp:
        string_arm=temp
        #stdscr.addstr(len(FRAMES)+9, 0, string_arm + '\t\t')
        publ.publish(string_arm)

    temp=''
    if abs(valfa[0]) > 10:
        temp='opening forearm'
        if valfa[0] < 0:
            temp='closing forearm'
    else:
        temp='forearm stopped'

    if string_arm1!=temp:
        string_arm1=temp
        #stdscr.addstr(len(FRAMES)+10, 0, string_arm1 + '\t\t')
        publ.publish(string_arm1)


    temp=''
    if Coord[5][2] > Coord[4][2]:
        temp='hand above elbow'
    else:
        temp='hand under elbow'

    if string_hand!=temp:
        string_hand=temp
        #stdscr.addstr(len(FRAMES)+11, 0, string_hand)
        publ.publish(string_hand)

def handing(publ):
    #TODO


if __name__ == '__main__':
    stdscr = curses.initscr()
    print_coord()
    atexit.register(exit_handler)
    rospy.init_node('kinect_tracking')

    gest_publ=gesture_publisher()
    centermass_publ=masterlocation_publisher()

    file_out = open('Database.txt', 'a')

    time_coord=[[[0,0,0] for t in range(MEDIANSIZE)] for i in range(FRAME_COUNT)]

    listener = tf.TransformListener()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        master=1

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
                #store filtered results
                Coord[f] = median_res
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angles()
        print_coord()
        #master's center of mass location
        position[1:4]=position[0:3]
        '''
        1  - /neck
        2  - /torso
        6  - /left_hip
        12 - /right_hip
        '''
        position[0]=(Coord[2][0]+ Coord[6][0]+ Coord[12][0]+ Coord[1][0])/4
        centermass_publ.publish(master, position[0])

        waving(gest_publ)

        calling(gest_publ)

        walk(gest_publ)

        hand(gest_publ)

        #forearm()
        #arm()
        #state()

        rate.sleep()
