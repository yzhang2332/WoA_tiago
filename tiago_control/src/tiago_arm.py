#! /usr/bin/env python 

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState


# initialize ros node
rospy.init_node('tiago_arm_controller')

# create a publisher
arm_pub = rospy.Publisher('/arm_controller/safe_command', JointTrajectory, queue_size=10)

tiago_joint_names = ['arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint']

def tiago_homing():
    # bring robot to the home position

    # create a JointTrajectory message for for the arm
    arm_homing_traj = JointTrajectory()
    arm_homing_traj.joint_names = tiago_joint_names

    arm_home_position = JointTrajectoryPoint() # home position
    arm_home_position.positions = [0, 0, 0, 0, 0, 0, 0] # joint angles in radians
    arm_home_position.time_from_start = rospy.Duration(1)

    arm_homing_traj.points.append(arm_home_position)

    rate = rospy.Rate(20)


    while pos_error < 1e-3:
        arm_pub.publish(arm_homing_traj)

        rospy.loginfo("Homing trajectory points are publishing...")

        rate.sleep()


def tiago_hand_shaking():
    # allows robot to handshake

def tiago_welcome():
    # welcome gesture

def tiago_waving():

    # create a JointTrajectory message for for the arm
    arm_waving_traj = JointTrajectory()
    arm_waving_traj.joint_names = tiago_joint_names

    arm_joint_pos1 = JointTrajectoryPoint()
    arm_joint_pos1.positions = [0, 0, 0, 0, 0, 0, 0] # joint angles in radians
    arm_joint_pos1.time_from_start = rospy.Duration(2)

    arm_joint_pos2 = JointTrajectoryPoint()
    arm_joint_pos2.positions = [0, 0, 0, 0, 0, 0, 0] # joint angles in radians
    arm_joint_pos2.time_from_start = rospy.Duration(3)
   
    num_cycle = 2

    for _ in range(num_cycle):
        arm_waving_traj.points.append(arm_joint_pos1)
        arm_waving_traj.points.append(arm_joint_pos2)

    rate = rospy.Rate(20) # Hz
    
    counter = 0

    while counter < num_cycle:
        # publish the trajectory points
        arm_pub.publish(arm_waving_traj)

        rospy.loginfo('Waving trajectory points are publishing...')

        rate.sleep()
        counter += 1


if __name__ == '__main__':
    try:
        tiago_homing() # homing
        rospy.sleep(3)

        tiago_waving() # waving
        rospy.sleep(3)

    except rospy.ROSInterruptException:
        pass

