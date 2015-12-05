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
vx=0
vy=0
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

    # Assemble do feature vector
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


    #save to file
    file_out = open('build_database.txt', 'a')
    file_out.write(str(feature_vector) + '\n')
    file_out.close()

    data_base.append(feature_vector)

    x_past = D_x
    y_past = D_y



def goodbye():
    rospy.loginfo('Exiting...')

if __name__ == '__main__':
    atexit.register(goodbye)
    rospy.init_node('hri_decision_node', anonymous=True)

    '''import Tkinter as tk

    def key(event):
        print event.keysym
        if event.keysym == 'Escape':
            root.destroy()
        print event.char

    root = tk.Tk()
    print "Press a key (Escape key to exit):"
    root.bind_all('<Key>', key)
    #root.withdraw()
    root.mainloop()'''

    rate = rospy.Rate(0.5)
    rospy.Subscriber('/gestures', gesture, new_gesture)
    rospy.Subscriber('/masterlocation', pose_msg, new_position)

    while not rospy.is_shutdown():
        rate.sleep()
