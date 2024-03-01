#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DepthImageProcessor:
    def __init__(self):

        rospy.init_node('depth_image_processor', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/xtion/depth/image_raw", Image, self.depth_callback)
        self.person_depth_sub = rospy.Subscriber("/pixel_to_track", JointState, self.person_depth_callback)
        self.pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)
        self.base_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.mobile_base_callback)
        self.depth_info = None
        self.person_loc = None

        self._hz = rospy.get_param('~hz', 10)
        self._forward_rate = rospy.get_param('~forward_rate', 0.1)
        self._backward_rate = rospy.get_param('~backward_rate', 0.1)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.1)
        self._angular = 0
        self._linear = 0
        self._curr_linear = 0
        self._curr_angular = 0
        self._linear_out = 0
        self._setpoint_x = 0
        self._setpoint_th = 0
        self.prev_error = 0
        #self.integral = 0

    

    def mobile_base_callback(self, msg):
        self._curr_linear = msg.twist.twist.linear.x
        self._curr_angular = msg.twist.twist.angular.z

        # kp = 1; kd = 1; 

        # current_time = rospy.get_time()
        # # Compute time step
        # dt = current_time - self.prev_time
        # # Update previous time
        # self.prev_time = current_time

        # linear_error = self._setpoint_x - self._curr_linear
        # diff_error = linear_error - self.prev_error
        # #self.integral += linear_error

        # self._linear_out = kp * linear_error + (kd * diff_error)/dt 

        # # Update previous error
        # self.prev_error = linear_error


    def person_depth_callback(self, msg):
        positions = msg.position
        found_person = positions[2]
        if found_person == 1:
            self.person_loc = [positions[0], positions[1]]
        else:
            self.person_loc = None
        # self.process_data()
        #print("positions x: {},y: {}".format(positions[0], positions[1]))


    def depth_callback(self, data):
        try:
            # Convert the depth image using the default passthrough encoding
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            
            # Convert to a Numpy array for easier manipulation
            depth_array = np.array(cv_image, dtype=np.float32)
            self.depth_info = depth_array
            self.process_data()

        except CvBridgeError as e:
            print(e)


    def process_data(self):
        # # Create a JointTrajectory message for the head
        # self.head_traj = JointTrajectory()
        # self.head_traj.joint_names = ['head_1_joint', 'head_2_joint']  # Tiago has two head joints
        # # Keep head position straight
        # self.head_pan = math.radians(0) # left(+) right(-) motion
        # self.head_tilt = math.radians(0) #  up(+) down(-) motion
        # # Define a trajectory point
        # self.traj_point = JointTrajectoryPoint()
        # self.traj_point.positions = [self.head_pan, self.head_tilt]  # Example position values for head joints
        # self.traj_point.time_from_start = rospy.Duration(1.5)  # Time duration for reaching the desired position

        # Add the trajectory point to the JointTrajectory message
        self.head_traj.points.append(self.traj_point)
        self.head_pub.publish(self.head_traj)
        #print('Head data is publishing...')
        def _get_twist(linear, angular):
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            return twist
        
        if self.depth_info is not None and self.person_loc is not None:
            #print('person_loc: x = {}, y = {}'.format(int(self.person_loc[0]), int(self.person_loc[1]))) 
            self.person_depth = self.depth_info[int(self.person_loc[1]), int(self.person_loc[0])]
            self.person_depth = self.person_depth/1000
            print('person_loc: x = {}, y = {}, depth = {} (m)'.format(int(self.person_loc[0]), int(self.person_loc[1]), self.person_depth))
            

            safe_distance = 1 # meter
            tolerance = 0.1 # sensor noise

            if (safe_distance - tolerance) < self.person_depth < (safe_distance + tolerance):
                self._linear = 0 # m/s
            elif self.person_depth > safe_distance + tolerance:
                self._linear = 0.35
            elif self.person_depth < safe_distance - tolerance:
                self._linear = -0.35
            # self._linear = 0
            self._angular = (320 - self.person_loc[0])* (1/500) 
            twist = _get_twist(self._linear, self._angular)
            self.pub_cmd.publish(twist)

 



def main():
    
    dip = DepthImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
