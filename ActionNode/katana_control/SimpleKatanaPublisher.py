#!/usr/bin/env python


# How to run:
#   - cd to katana_control. make
#        OR
#   - rosmake katana_control
#        Then
#   - rosrun katana_control SimpleKatanaPublisher.py
#

'''
JointMovementActionGoal
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    actionlib_msgs/GoalID goal_id
      time stamp
      string id
    katana_msgs/JointMovementGoal goal
      sensor_msgs/JointState jointGoal
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort
'''

joint_names = ['katana_motor1_pan_joint', 'katana_motor2_lift_joint', 'katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint', 'katana_r_finger_joint', 'katana_l_finger_joint']



# Waving:

position_sequence = [[-1.85,1.57,0.175,-0.35,-1.57,0.175,0.175],[-1.85,1.57,0.175,-0.35,-1.57,0.175,0.175],[-1.85,1.57,0.175,-0.35,-1.57,0.175,0.175]]
delay_sequence = [1.5,1.25,1.25]



import rospy
from katana_msgs.msg import JointMovementActionGoal as jmag

def katana_ctrl():
    pub = rospy.Publisher('/katana_arm_controller/joint_movement_action/goal', jmag, queue_size=10)
    rospy.init_node('katana_arm_our_controler', anonymous=True)
    jmag_msg = jmag()
    jmag_msg.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
    jmag_msg.goal_id.stamp = rospy.Time.now()
    jmag_msg.goal_id.id = 'SimpleKatana test - ' + str(jmag_msg.header.stamp.secs) + ' . ' + str(jmag_msg.header.stamp.nsecs)
    jmag_msg.goal.jointGoal.name=joint_names
    jmag_msg.goal.jointGoal.position = [-1.85,1.57,0.175,-0.35,-1.57,0.175,0.175]

    rate = rospy.Rate(0.9)
    rate.sleep()
    while not rospy.is_shutdown():
        jmag_msg.header.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        jmag_msg.goal_id.stamp = rospy.Time.now()
        jmag_msg.goal_id.id = 'SimpleKatana test - ' + str(jmag_msg.header.stamp.secs) + ' . ' + str(jmag_msg.header.stamp.nsecs)
        jmag_msg.goal.jointGoal.position = [jmag_msg.goal.jointGoal.position[0],jmag_msg.goal.jointGoal.position[1],-jmag_msg.goal.jointGoal.position[2],-jmag_msg.goal.jointGoal.position[3],jmag_msg.goal.jointGoal.position[4],jmag_msg.goal.jointGoal.position[5],jmag_msg.goal.jointGoal.position[6]]
        rospy.loginfo(jmag_msg)
        pub.publish(jmag_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        katana_ctrl()
    except rospy.ROSInterruptException:
        pass
