#!/usr/bin/env python


# How to run:
#   - cd to scout_control. make
#        OR
#   - rosmake scout_control
#        Then
#   - rosrun scout_control scout_simple_goal_publisher.py
#

'''
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
    #A representation of pose in free space, composed of postion and orientation.
    Point position
        # This contains the position of a point in free space
        float64 x
        float64 y
        float64 z
    Quaternion orientation
        # This represents an orientation in free space in quaternion form.
        float64 x
        float64 y
        float64 z
        float64 w
'''

import time
import rospy
import tf
from geometry_msgs.msg import PoseStamped as ps
from geometry_msgs.msg  import Point
from geometry_msgs.msg  import Pose
from geometry_msgs.msg  import Quaternion

import std_msgs.msg


def scout_ctrl():

    pub = rospy.Publisher('/move_base_simple/goal', ps, queue_size=5)
    time.sleep(1)
    message = ps()

    message.header.stamp =  rospy.Time.now()

    message.pose=Pose()
    #message.pose.position = Point()
    message.pose.position.x = 1
    message.pose.position.y = 1
    message.pose.position.z = 0

    message.pose.orientation = Quaternion()
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    message.pose.orientation.x = quat[0]
    message.pose.orientation.y = quat[1]
    message.pose.orientation.z = quat[2]
    message.pose.orientation.w = quat[3]



    rate = rospy.Rate(0.25) # 10hz

    rospy.loginfo(message)
    pub.publish(message)
    rate.sleep()

    while not rospy.is_shutdown():

        h.stamp = rospy.Time.now()

        message.pose.position.x = -message.pose.position.x
        message.pose.position.y = -message.pose.position.y
        message.pose.position.z = 0

        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('scout_control', anonymous=True)
        time.sleep(1)
        scout_ctrl()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
