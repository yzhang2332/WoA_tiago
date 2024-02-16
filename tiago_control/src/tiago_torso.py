#! /usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import math
import time


def control_tiago_torso():
    # initialize ros node
    rospy.init_node('tiago_torso')

    # create a publisher
    torso_pub = rospy.Publisher('/torso_controller/safe_command', JointTrajectory, queue_size=10)
        
    torso_traj = JointTrajectory()
    torso_traj.joint_names = ['torso_lift_joint']

    traj_point = JointTrajectoryPoint()
    traj_point.positions = [0.3] # range from 0 to 0.350 meter

    traj_point.time_from_start = rospy.Duration(1.0)

    torso_traj.points.append(traj_point)

    rate = rospy.Rate(20) # Hz

    while not rospy.is_shutdown():

        # publish traj_point on topic
        torso_pub.publish(torso_traj)
        rospy.loginfo('torso trajectory point published...')
        rate.sleep()

if __name__== '__main__':
    try:
        control_tiago_torso()
    except rospy.ROSInterruptException:
        pass