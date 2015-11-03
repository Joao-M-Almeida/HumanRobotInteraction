#!/usr/bin/env python


# How to run:
#   - cd to katana_control. make
#   - rosrun katana_control SimpleKatanaPublisher.py
#

import rospy
from katana_msgs.msg import JointMovementActionGoal as jmag
from katana_msgs.msg import JointMovementGoal as jmg

import std_msgs.msg
from actionlib_msgs.msg import GoalID


def katana_ctrl():
    pub = rospy.Publisher('/katana_arm_controller/joint_movement_action/goal', jmag, queue_size=10)
    rospy.init_node('katana_arm_our_controler', anonymous=True)
    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    g = GoalID()
    g.stamp = rospy.Time.now()
    g.id = 'SimpleKatana test - ' + str(h.stamp.secs) + ' . ' + str(h.stamp.nsecs)
    goal=jmg()
    goal.jointGoal.name=['katana_motor1_pan_joint']
    goal.jointGoal.position = [1]
    jmag_msg = jmag()
    jmag_msg.header = h
    jmag_msg.goal_id = g
    jmag_msg.goal = goal
    rate = rospy.Rate(0.25) # 10hz
    while not rospy.is_shutdown():
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        g.stamp = rospy.Time.now()
        g.id = 'SimpleKatana test - ' + str(h.stamp.secs) + ' . ' + str(h.stamp.nsecs)
        goal.jointGoal.position = [-goal.jointGoal.position[0]]
        rospy.loginfo(jmag_msg)
        pub.publish(jmag_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        katana_ctrl()
    except rospy.ROSInterruptException:
        pass
