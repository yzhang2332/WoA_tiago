#! /usr/bin/env python
import rospy

rospy.init_node("python_node")
rate = rospy.Rate(2)
while not rospy.is_shutdown():
    print("Hello, Python Node")
    rate.sleep()