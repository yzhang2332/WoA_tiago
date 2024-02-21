#! /usr/bin/env python

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib
import numpy as np

def joint_state_callback(msg):
    
    position_error = msg.error.positions
    error_sum = np.sum([position_error]) 

def tiago_safe_config():
    rospy.init_node('tiago_safe_config')
    # action client with FollowJointTrajectoryAction server running on this topic /arm_controller/follow_joint_trajectory
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    print('Server is up...')
#    arm_pub = rospy.Publisher('/arm_controller/safe_command', JointTrajectory, queue_size=10)
    rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, joint_state_callback)
    tiago_joints_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    goal_pos = FollowJointTrajectoryGoal()
    goal_pos.trajectory.joint_names = tiago_joints_names

    safe_config = JointTrajectoryPoint()
    safe_config.positions = [0.32333166178803174, -1.395578991928086, -0.31773010781619595, 1.8215411533599226, -1.50814934991959567, 1.4106668422, 0.128606835]
    
    safe_config.time_from_start = rospy.Duration(3)
    
    goal_pos.trajectory.points.append(safe_config)
    client.send_goal(goal_pos)
    print('Goal position sent ...')

    client.wait_for_result()

    print('Tiago is now in the safe configuration...')

if __name__ == '__main__':
    try:
        tiago_safe_config()
    except rospy.ROSInterruptException:
        pass