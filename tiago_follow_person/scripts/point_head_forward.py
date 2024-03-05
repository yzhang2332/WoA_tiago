#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import math

# trajectory_msgs: messages for robot joint trajectory

def control_tiago_head():
    # Initialize ROS node
    rospy.init_node('tiago_head_controller') # node_name = tiago_head_controller

    # Define ROS publisher for sending head trajectory commands
    # rospy.Publisher('topic_name', String, queue_size=10)
    head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

    # Create a JointTrajectory message for the head
    head_traj = JointTrajectory()
    head_traj.joint_names = ['head_1_joint', 'head_2_joint']  # Tiago has two head joints
    head_pan = math.radians(0) # left(+) right(-) motion
    head_tilt = math.radians(0) #  up(+) down(-) motion
    # Define a trajectory point
    traj_point = JointTrajectoryPoint()
    traj_point.positions = [head_pan, head_tilt]  # Example position values for head joints
    traj_point.time_from_start = rospy.Duration(1.5)  # Time duration for reaching the desired position

    # Add the trajectory point to the JointTrajectory message
    head_traj.points.append(traj_point)

    rate = rospy.Rate(20) # Hz

    while not rospy.is_shutdown():
        # Publish the head trajectory command
        head_pub.publish(head_traj)

        # rospy.loginfo("Head trajectory command published...")

        rate.sleep()

if __name__ == '__main__':
    try:
        control_tiago_head()
    except rospy.ROSInterruptException:
        pass