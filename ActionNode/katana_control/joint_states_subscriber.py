#!/usr/bin/env python

# Subscribe to /joint_states and print to terminal
# Receives sensor_msgs/JointState

import rospy
from sensor_msgs.msg import JointState as js

def callback(data):
    rospy.loginfo('Joints: ' + str(data.name))
    rospy.loginfo('positions: ' + str(data.position))
    rospy.loginfo('velocities: ' + str(data.velocity))



def joint_states_listener():

    rospy.init_node('joint_states_listener', anonymous=True)

    rospy.Subscriber('/joint_states', js, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    joint_states_listener()
