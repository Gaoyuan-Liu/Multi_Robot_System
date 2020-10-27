#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

def panda_position_command_talker():
    pub = rospy.Publisher('/panda/endeffector_command', Float64MultiArray, queue_size=100)
    rospy.init_node('panda_position_command_talker', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        position_command = Float64MultiArray()
        position_command.data = [0.25, 0.05, 0.40]
        #position_command.data[2] = 0.05
        #position_command.data[3] = 0.60
        #position_command.header.frame_id = "world"
        pub.publish(position_command)
        rate.sleep()

if __name__ == '__main__':
    try:
        panda_position_command_talker()
    except rospy.ROSInterruptException:
        pass
