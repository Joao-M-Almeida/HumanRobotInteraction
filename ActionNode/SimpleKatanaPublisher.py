header:
  seq: 118
  stamp:
    secs: 1445941280
    nsecs: 982897301
  frame_id: ''
goal_id:
  stamp:
    secs: 1445941280
    nsecs: 982898198
  id: /katana_teleop_key-119-1445941280.982898198
goal:
  jointGoal:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: ''
    name: ['katana_motor1_pan_joint']
    position: [3.0481766367742633]
    velocity: []
    effort: []


#!/usr/bin/env python
# license removed for brevity
import rospy
from katana_msgs

def talker():
    pub = rospy.Publisher(' /katana_arm_controller/joint_movement_action/goal', String, queue_size=10)
    rospy.init_node('katana_arm_our_controler', anonymous=True)
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
