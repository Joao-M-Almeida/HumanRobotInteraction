#!/usr/bin/env python


# How to run:
#   - cd to katana_control. make
#        OR
#   - rosmake katana_control
#        Then
#   - rosrun katana_control SimpleKatanaPublisher.py
#

'''
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
control_msgs/JointTrajectoryGoal goal
  trajectory_msgs/JointTrajectory trajectory
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string[] joint_names
    trajectory_msgs/JointTrajectoryPoint[] points
      float64[] positions
      float64[] velocities
      float64[] accelerations
      float64[] effort
      duration time_from_start
'''


import rospy
from control_msgs.msg import JointTrajectoryActionGoal as jtag
from control_msgs.msg import JointTrajectoryGoal as jtag_goal
from trajectory_msgs.msg import JointTrajectoryPoint as jtp
from trajectory_msgs.msg import JointTrajectory as jt


import std_msgs.msg
from actionlib_msgs.msg import GoalID


from sensor_msgs.msg import JointState as js

read_joint_states = False
read_joint_names = []
joint_pos = []
joint_vels = []



def callback(data):
    global read_joint_names
    global read_joint_names
    global joint_pos
    global joint_vels
    global read_joint_states
    rospy.loginfo('Just read /joint_states')
    read_joint_names = data.name
    joint_pos = data.position
    joint_vels = data.velocity
    read_joint_states = True
    #rospy.loginfo('names: ' + str(read_joint_names))



def joint_states_listener():
    global read_joint_states
    rospy.Subscriber('/joint_states', js, callback)
    if read_joint_states :
        return


def katana_traject_ctrl():
    global read_joint_names
    rospy.loginfo(str(read_joint_names))

    pub = rospy.Publisher('/katana_arm_controller/joint_trajectory_action/goal', jtag, queue_size=10)

    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work

    g = GoalID()
    g.stamp = rospy.Time.now()
    g.id = 'SimpleKatana trajectory test - ' + str(h.stamp.secs) + ' . ' + str(h.stamp.nsecs)

    jtag_msg = jtag()
    jtag_msg.header = h
    jtag_msg.goal_id = g

    # Create points
    p0 = jtp()
    p0.positions = joint_pos;
    p0.time_from_start = rospy.Duration(0)
    p1 = jtp()
    p1.positions = [0,0,0,0,0,0,0]
    p1.time_from_start = rospy.Duration(3)


    # Create the Trajectory Goal
    traject = jt()
    '''
    0: katana_motor1_pan_joint
    1: katana_motor2_lift_joint
    2: katana_motor3_lift_joint
    3: katana_motor4_lift_joint
    4: katana_motor5_wrist_roll_joint
    '''
    traject.joint_names = read_joint_names # ['katana_motor1_pan_joint','katana_motor2_lift_joint','katana_motor3_lift_joint','katana_motor4_lift_joint','katana_motor5_wrist_roll_joint']
    traject.points = [p0,p1]


    goal = jtag_goal()
    goal.trajectory=traject

    jtag_msg.goal = goal



    rate = rospy.Rate(0.1) # 10hz


    while not rospy.is_shutdown():
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        g.stamp = rospy.Time.now()
        g.id = 'SimpleKatana trajectory test - ' + str(h.stamp.secs) + ' . ' + str(h.stamp.nsecs)
        rospy.loginfo(jtag_msg)
        pub.publish(jtag_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('katana_arm_our_controler', anonymous=True)
        joint_states_listener()
        katana_traject_ctrl()
    except rospy.ROSInterruptException:
        pass
