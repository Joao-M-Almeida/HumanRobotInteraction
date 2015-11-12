#!/usr/bin/env python

import rospy
import atexit
import std_msgs.msg
from ros_skeleton_tracker.msg import pose_msg

def exit_handler():
    print 'Exiting!'

if __name__ == '__main__':
    atexit.register(exit_handler)
    rospy.init_node('gesture_publisher')
    pub = rospy.Publisher('/gestures',pose_msg, queue_size=10)
    sender_msg = pose_msg()

    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    sender_msg.header = h
    sender_msg.gesture='Hello World!'

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        h.stamp = rospy.Time.now()
        sender_msg.header = h
        pub.publish(sender_msg)
        rate.sleep()
