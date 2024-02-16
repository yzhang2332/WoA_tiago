#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def tiago_sensors_callback(msg):
    
    each_joint_position = msg.position
    each_joint_velocity = msg.velocity
    each_joint_effort = msg.effort
    
    joint_name = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'head_1_joint', 'head_2_joint', 'torso_lift_joint'] 
    
    #index = msg.name.index(joint_name[-1])
    #position = msg.position[index]
    #rospy.loginfo(position)


def main():
    rospy.init_node('joint_state_subscriber')

    rospy.Subscriber('/joint_states', JointState, tiago_sensors_callback)
    # spin to prevent the node from exiting
    rospy.spin()

if __name__ == '__main__':
    main()

