#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(data):
    trans = data.pose.pose.position
    orientation_q = data.pose.pose.orientation
    parent_frame = data.header.frame_id
    child_frame = data.child_frame_id
    print("parent = %s, child = %s" % (parent_frame, child_frame))
    # rospy.loginfo(rospy.get_caller_id() + "I heard translation: [%f, %f, %f]",
    #               trans.x, trans.y, trans.z)
    # rospy.loginfo(rospy.get_caller_id() + "I heard orientation: [%f, %f, %f, %f]",
    #               orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/mobile_base_controller/odom", Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
