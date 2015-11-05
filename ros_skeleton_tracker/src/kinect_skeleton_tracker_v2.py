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



def exit_handler():
    curses.endwin()
    file_out.close()
    print 'Exiting!'

def print_coord():
    for f in range(0,len(FRAMES)):
        string=str(FRAMES[f]) + '\t->\t' + str(['%0.3f' % elem for elem in Coord[f]]) + '\t'

        stdscr.addstr(f, 0, string)
        #print FRAMES[f] , '->', Coord[f]

    angle_string='Angle alfa\t->\t' + str(alfa)
    stdscr.addstr(len(FRAMES), 0, angle_string)

    angle_string='Angle beta\t->\t' + str(beta)
    stdscr.addstr(len(FRAMES)+1, 0, angle_string)

    angle_string='Angle gama\t->\t' + str(gama)
    stdscr.addstr(len(FRAMES)+2, 0, angle_string)

    angle_string='Angle valfa\t->\t' + str(valfa)
    stdscr.addstr(len(FRAMES)+3, 0, angle_string)

    angle_string='Angle vbeta\t->\t' + str(vbeta)
    stdscr.addstr(len(FRAMES)+4, 0, angle_string)

    angle_string='Angle vgama\t->\t' + str(vgama)
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
    alfa[0]=math.degrees(math.acos(dotproduct(lsh_elb, lhan_elb) / (length(lsh_elb) * length(lhan_elb))))
    alfa[0]=int(alfa[0])
    ##arm##
    #angle between arm and spine
    torso_neck=[Coord[2][0]-Coord[1][0], Coord[2][1]-Coord[1][1], Coord[2][2]-Coord[1][2]]
    elb_lsh=[Coord[4][0]-Coord[3][0], Coord[4][1]-Coord[3][1], Coord[4][2]-Coord[3][2]]
    beta[0]=math.degrees(math.acos(dotproduct(torso_neck, elb_lsh) / (length(torso_neck) * length(elb_lsh))))
    beta[0]=int(beta[0])
    #angle between arm and chest
    sholderr_sholderl=[Coord[9][0]-Coord[3][0], Coord[9][1]-Coord[3][1], Coord[9][2]-Coord[3][2]]
    elb_shl=[Coord[4][0]-Coord[3][0], Coord[4][1]-Coord[3][1], Coord[4][2]-Coord[3][2]]
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

# def waving():
    # angle_string='\t\t'
    # if alfa[0] > 70 and alfa[0]< 120:
        # if beta[0] > 70 and beta[0]< 120:
            # if abs(valfa) < 0.001:
                # #if abs(valfa) > 1.5*abs(vbeta):
                # angle_string='waving'
    # stdscr.addstr(len(FRAMES)+3, 0, angle_string)

def forearm():
    global valfa
    global state_forearm
    global flag2
    if abs(valfa[0]) < 3:
        state_forearm='forearm stopped\t\t'
        flag2=flag2+1
    else:
        if valfa[0] < 0:
            state_forearm='forearm closing\t\t'
            flag2=0
        if valfa[0] > 0:
            state_forearm='forearm opening\t\t'
            flag2=0

    #string1='The forearm is ' + str(state_forearm)
    stdscr.addstr(len(FRAMES)+9, 0, state_forearm)
    stdscr.refresh()


def arm():
    global vbeta, vgama
    global flag, flag1
    global state_arm, state_arm2
    if abs(vbeta[0]) < 3:
        state_arm='arm stopped vertical\t\t'
        flag=flag+1
    else:
        if vbeta[0] > 0:
            state_arm='arm moving up\t\t'
            flag=0
        if vbeta[0] < 0:
            state_arm='arm moving down\t\t'
            flag=0

    if abs(vgama[0]) < 3:
        state_arm2='arm stopped horizontal\t\t'
        flag1=flag1+1
    else:
        if vgama[0] > 0:
            state_arm2='arm opening\t\t'
            flag1=0
        if vgama[0] < 0:
            state_arm2='arm closing\t\t'
            flag1=0

    stdscr.addstr(len(FRAMES)+7, 0, state_arm)
    stdscr.addstr(len(FRAMES)+8, 0, state_arm2)
    stdscr.refresh()

def state():
    global vgama, vbeta, valfa, stat
    global flag, flag1, flag2
    global file_out
    global state_arm, state_arm2, state_forearm
    if abs(vgama[0]) < 3 and flag1==1:
        stat=stat+1
        frase='state '+ str(stat) + ':\n' + str(state_arm2) + '\n' + str(state_arm) + '\n' + str(state_forearm) + '\n'
        file_out.write(frase)
    if abs(vbeta[0]) < 3 and flag==1:
        stat=stat+1
        frase='state '+ str(stat) + ':\n' + str(state_arm2) + '\n' + str(state_arm) + '\n' + str(state_forearm) + '\n'
        file_out.write(frase)
    if abs(valfa[0]) < 3 and flag2==1:
        stat=stat+1
        frase='state '+ str(stat) + ':\n' + str(state_arm2) + '\n' + str(state_arm) + '\n' + str(state_forearm) + '\n'
        file_out.write(frase)

    #string2='The arm is ' + str(state_arm) + ' and ' + str(state_arm2)




if __name__ == '__main__':
    stdscr = curses.initscr()
    print_coord()
    atexit.register(exit_handler)
    file_out = open('Database.txt', 'a')
    rospy.init_node('kinect_tracking')

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

        print_coord()
        #waving()
        forearm()
        arm()
        state()


        rate.sleep()
