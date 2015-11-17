#!/usr/bin/env python

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

import rospy
from katana_msgs.msg import JointMovementActionGoal as jmag
from std_msgs.msg import String

joint_names = ['katana_motor1_pan_joint', 'katana_motor2_lift_joint', 'katana_motor3_lift_joint', 'katana_motor4_lift_joint', 'katana_motor5_wrist_roll_joint', 'katana_r_finger_joint', 'katana_l_finger_joint']

pub = rospy.Publisher('/katana_arm_controller/joint_movement_action/goal', jmag, queue_size=10)

default_position = [-0.28,2,-2,-1.7,0,0,0]

calibration_position = [-2.9,2.1,-2.15,-1.91,-2.87,0.29,0.29]

def waving():
    position_sequence = [[-1.85,1.57,0.175,-0.35,-1.57,0.175,0.175], [-1.85,1.57,-0.175,0.35,-1.57,0.175,0.175], [-1.85,1.57,0.175,-0.35,-1.57,0.175,0.175], [-1.85,1.57,-0.175,0.35,-1.57,0.175,0.175], default_position]
    delay_sequence = [5,1.3,1.3,1.3,5]
    execute_movement(position_sequence, delay_sequence)

def clapping():
    position_sequence = [[-1.85,1.57,1.57,1.57,-1.57,0.3,0.3], [-1.85,1.57,1.57,1.57,-1.57,-0.44,-0.44], [-1.85,1.57,1.57,1.57,-1.57,0.3,0.3], [-1.85,1.57,1.57,1.57,-1.57,-0.44,-0.44], [-1.85,1.57,1.57,1.57,-1.57,0.3,0.3], default_position]
    delay_sequence = [10,2.5,2.5,2.5,2.5,8]
    execute_movement(position_sequence, delay_sequence)

def high_five():
    position_sequence = [[-0.28,1.57,0,0,0,-0.2,-0.2], [-0.28,1.4,-2.2,0,0,-0.2,-0.2], default_position]
    delay_sequence = [6,6,3]
    execute_movement(position_sequence, delay_sequence)

def low_five():
    position_sequence = [[-0.28,1.4,-2.2,0,0,-0.2,-0.2], [-0.28,1.57,0,0,0,-0.2,-0.2], default_position]
    delay_sequence = [2.5,6.5,6]
    execute_movement(position_sequence, delay_sequence)

def execute_movement(position_sequence, delay_sequence):
    if len(position_sequence) != len(delay_sequence):
        rospy.logerr("KATANA MOVEMENT ERROR")
    jmag_msg = jmag()
    for index, position in enumerate(position_sequence):
        jmag_msg.header.stamp = rospy.Time.now()
        jmag_msg.goal_id.stamp = rospy.Time.now()
        jmag_msg.goal.jointGoal.name=joint_names
        jmag_msg.goal_id.id = 'Katana Command - ' + str(jmag_msg.header.stamp.secs) + ' . ' + str(jmag_msg.header.stamp.nsecs)
        jmag_msg.goal.jointGoal.position=position
        rospy.loginfo(jmag_msg)
        rospy.loginfo("Rate : " + str(1/delay_sequence[index]))
        pub.publish(jmag_msg)
        rate = rospy.Rate(1.0/delay_sequence[index])
        rate.sleep()

def command_process(data):
    rospy.loginfo('command received: ' + str(data))
    if data.data == 'wave':
        waving()
    elif data.data == 'low_five':
        low_five()
    elif data.data == 'high_five':
        high_five()
    elif data.data == 'clap':
        clapping()
    elif data.data == 'default':
        execute_movement([default_position], [10])
    elif data.data == 'calibration':
        execute_movement([calibration_position], [10])


if __name__ == '__main__':
    try:
        rospy.init_node('katana_controller', anonymous=True)
        rospy.Subscriber('/katana_commands/', String, command_process)
        rate = rospy.Rate(0.5)
        rate.sleep()
        execute_movement([default_position], [10])
        #execute_movement([[-0.28,1.57,0,0,0,-0.2,-0.2]],[10])
        #low_five()
        #high_five()
        #waving()
        #clapping()
        #execute_movement([default_position], [10])
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
