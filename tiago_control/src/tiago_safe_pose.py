#! /usr/bin/env python

import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib
import numpy as np

def joint_state_callback(msg):
    
    position_error = msg.error.positions
    error_sum = np.sum([position_error]) 
    rospy.loginfo('error sum: %s', error_sum)
#    if error_sum < 1e-5:
#        rospy.loginfo('Node exited.. Done!!!')
#        rospy.signal_shutdown('Error is below threshold..')

def tiago_safe_config():
    rospy.init_node('tiago_safe_config')
    # action client
    client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

#    arm_pub = rospy.Publisher('/arm_controller/safe_command', JointTrajectory, queue_size=10)
    rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, joint_state_callback)
    tiago_joints_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

    goal_pos = FollowJointTrajectoryGoal()
    goal_pos.trajectory.joint_names = tiago_joints_names

    safe_config = JointTrajectoryPoint()
    safe_config.positions = [0.32333166178803174, -1.392816991928086, -0.32336010781619595, 1.6018415533599226, -1.5849634991959567, -0.08496649918854628, 0.09357908261784606]
    safe_config.time_from_start = rospy.Duration(1)
    
    goal_pos.trajectory.points.append(safe_config)

    client.send_goal(goal_pos)

    client.wait_for_result()

#    rate = rospy.Rate(20)
#    while not rospy.is_shutdown():
#        arm_pub.publish(safe_pos_traj)
#        rate.sleep()
    rospy.loginfo('Tiago Safe Configuration...')

if __name__ == '__main__':
    try:
        tiago_safe_config()
    except rospy.ROSInterruptException:
        pass
