#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from math import pi
# import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class PersonFollower:

    def __init__(self):

        rospy.init_node('person_follower', anonymous=True)

        self.bridge = CvBridge()
        self.depth_image_sub = rospy.Subscriber("/xtion/depth/image_raw", Image, self.depth_image_callback)
        self.person_target_sub = rospy.Subscriber("/pixel_to_track", JointState, self.person_target_callback)
        self.pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

        self.follow_flag_sub = rospy.Subscriber("/follow_person_flag", Bool, self.follow_flag_callback)
        self.obstacle_dir_sub = rospy.Subscriber("/obstacle_dir", JointState, self.obstacle_dir_callback)
        self.recovering_pub = rospy.Publisher('/recovering', Bool, queue_size=1)

        self._hz = rospy.get_param('~hz', 10)
        self._forward_rate = rospy.get_param('~forward_rate', 0.1)
        self._backward_rate = rospy.get_param('~backward_rate', 0.1)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.1)

        self.follow_flag = True               # this listens to the master file
        # self.obstacle_dir = [0, 0, 0, 0]      # this listens to the obstacle_checker node
        self.obstacle_flag = False
        self.recovering = False

        self.depth_matrix = None
        self.person_loc = None

        self.zero_depth_count = 0
        self.count_thres = 15

        self._linear = 0
        self._angular = 0

        ### define constants ###
        self.max_dist = 1.3          # meters
        self.min_dist = 1.2          # meters
        self.max_rot_diff = 50       # pixels

        self.forward = 0.35          # m/s
        self.backward = -0.2         # m/s
        self.rot_scaling = float(1/700)

        self.subarr_size = 10       # sub-matrix of the depth matrix

        # self._curr_linear = 0
        # self._curr_angular = 0
        # self._linear_out = 0
        # self._setpoint_x = 0
        # self._setpoint_th = 0
        # self.prev_error = 0
        # self.integral = 0

        # self.dist_tol = 0.15         # sensor noise


    def follow_flag_callback(self, msg: Bool):
        # from master file
        self.follow_flag = msg.data


    def obstacle_dir_callback(self, msg: JointState):
        if not self.follow_flag:
            return
        # from obstacle_checker node
        if self.obstacle_flag:
            return
        obstacle_dir = msg.position
        # print(obstacle_dir)
        all_zeros = all(x == 0 for x in obstacle_dir)
        if all_zeros:
            self.obstacle_flag = False
        else:
            self.obstacle_flag = True
            self.recover_from_obstacle(obstacle_dir)
            self.obstacle_flag = False


    def person_target_callback(self, msg):
        # Frequency of /pixel_to_track topic ~ 11 Hz
        positions = msg.position
        found_person = positions[2]
        if found_person == 1:
            self.person_loc = [positions[0], positions[1]]
        else:
            self.person_loc = None
        # self.process_data()
        #print("positions x: {},y: {}".format(positions[0], positions[1]))


    def depth_image_callback(self, data):
        # Frequency of /xtion/depth/image_raw topic = 30 Hz

        # publish the recovering flag at 30 Hz
        self.recovering_pub.publish(self.recovering)

        if self.follow_flag:
            try:
                # Convert the depth image using the default passthrough encoding
                cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                
                # Convert to a Numpy array for easier manipulation
                depth_array = np.array(cv_image, dtype=np.float32)
                self.depth_matrix = depth_array
                self.process_data()

            except CvBridgeError as e:
                self.depth_matrix = None
                print(e)
        else:
            rospy.loginfo("NO PERMISSION!")


    def get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist
    

    def translate(self, vel):

        rospy.loginfo("Translating")
        
        translate_duration = 2.0    # seconds
        start = rospy.get_time()
        now = rospy.get_time()

        diff = now-start
        while diff < translate_duration:
            cmd = self.get_twist(vel, 0.0)
            self.pub_cmd.publish(cmd)
            now = rospy.get_time()
            diff = now-start
    

    def rotate(self, angle):

        rospy.loginfo("Rotating")

        turn_duration = 3.0                         # seconds
        buffer = 0.0           # seconds
        turn_rate = angle / turn_duration   # rad/s [clockwise]

        start = rospy.get_time()
        now = rospy.get_time()

        diff = now-start
        while diff < turn_duration + buffer:
            cmd = self.get_twist(0.0, turn_rate)
            self.pub_cmd.publish(cmd)
            now = rospy.get_time()
            diff = now-start


    def recover_from_obstacle(self, obstacle_dir):
        
        # set to true at the start
        self.recovering = True

        print("="*20)
        print("inside the recovery function")
        print("="*20)

        print(obstacle_dir)
        print("Obstacle flag = %s" % self.obstacle_flag)

        # rospy.sleep(3.0)
        
        dir = obstacle_dir  # this is a list with at least 1 non-zero element

        # first check sides
        if dir[1] == 1:
            # rotate right
            self.rotate(-pi/8)
            # self.translate(0.2)
        if dir[3] == 1:
            # rotate left
            self.rotate(pi/8)
            # self.translate(0.2)
        
        # now, move linearly
        if dir[0] == 1:
            # move backwards
            self.translate(-0.2)
        if dir[2] == 1:
            # move forwards
            self.translate(0.2)

        print("="*20)
        print("Finished --- the recovery function")
        print("="*20)

        # set to false when finished
        self.recovering = False



    def process_data(self):

        if self.depth_matrix is not None and self.person_loc is not None:

            #print('person_loc: x = {}, y = {}'.format(int(self.person_loc[0]), int(self.person_loc[1]))) 
            # self.person_depth = self.depth_matrix[int(self.person_loc[1]), int(self.person_loc[0])]

            self.person_depth = self.get_mean_of_subarray(int(self.person_loc[0]), int(self.person_loc[1]))
            self.person_depth = self.person_depth / 1000
            diff_px = 320 - self.person_loc[0]
            # rospy.loginfo('person_loc: x = {}, y = {}, depth = {} m, diff_px = {}'.format(int(self.person_loc[0]), 
            #                                                                               int(self.person_loc[1]), 
            #                                                                               self.person_depth,
            #                                                                               diff_px))
            # set linear velocity (assuming non-zero depth)
            if self.person_depth > self.max_dist:
                self._linear = self.forward         # forward
                # self._linear = (self.person_depth - self.max_dist)*1.5
                self.zero_depth_count = 0
            elif 0 < self.person_depth < self.min_dist:
                self._linear = self.backward        # backward
                self.zero_depth_count = 0
            elif self.min_dist < self.person_depth < self.max_dist:
                self._linear = 0.0                  # within comfortable range
                self.zero_depth_count = 0
            

            if self.person_depth == 0.0:
                # depth is 0
                self.zero_depth_count += 1
                # stop moving only if have accumulated a number of zero depths (meaning person not found)
                if self.zero_depth_count > self.count_thres:
                    self._linear = 0.0

            
            # set angular velocity
            if abs(diff_px) > self.max_rot_diff:
                self._angular = diff_px * self.rot_scaling
            else:
                self._angular = 0.0
                
            # # set angular velocity
            # if diff_px > self.max_rot_diff:
            #     self._angular = 0.35
            # elif diff_px < -self.max_rot_diff:
            #     self._angular = -0.35
            # else:
            #     self._angular = 0.0
            
            # # do not move linearly if rotating
            # if self._angular > 0.0:
            #     self._linear = 0.0
                
            twist = self.get_twist(self._linear, self._angular)
            
            # 2 conditions to satisfy before sending base vel command
            # 1. There's no reported obstacles nearby from the collision_checker node
            # 2. The master file is allowing this to run
            if not self.obstacle_flag and self.follow_flag:
                rospy.loginfo("Flag = %s, All clear, sending base command now!" % self.obstacle_flag)
                self.pub_cmd.publish(twist)
            # else:
            #     if self.obstacle_flag:
            #         print("Obstacle detected!")
            #     if not self.follow_flag:
            #         print("Master file not permitting!")



    def get_mean_of_subarray(self, x, y):

        # Calculate half the dimensions to make the subarray centered around (x, y)
        half_size = self.subarr_size // 2
        
        # Determine the boundaries of the subarray
        start_x = max(x - half_size, 0)
        end_x = min(x + half_size + 1, self.depth_matrix.shape[1])
        start_y = max(y - half_size, 0)
        end_y = min(y + half_size + 1, self.depth_matrix.shape[0])
        
        # Extract the subarray
        subarray = self.depth_matrix[start_y:end_y, start_x:end_x]
        
        # Calculate and return the mean of the subarray
        return np.mean(subarray)


    # def mobile_base_callback(self, msg):
    #     self._curr_linear = msg.twist.twist.linear.x
    #     self._curr_angular = msg.twist.twist.angular.z

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


    # def process_data(self):

    #     def get_twist(linear, angular):
    #         twist = Twist()
    #         twist.linear.x = linear
    #         twist.angular.z = angular
    #         return twist

    #     if self.depth_matrix and self.person_loc:

    #         #print('person_loc: x = {}, y = {}'.format(int(self.person_loc[0]), int(self.person_loc[1]))) 
    #         self.person_depth = self.depth_matrix[int(self.person_loc[1]), int(self.person_loc[0])]
    #         self.person_depth = self.person_depth/1000
    #         print('person_loc: x = {}, y = {}, depth = {} (m)'.format(int(self.person_loc[0]), int(self.person_loc[1]), self.person_depth))

    #         safe_distance = 1.0 # meter
    #         dist_tol = 0.15 # sensor noise

    #         rot_tol = 20   # pixels
            

    #         if (safe_distance - dist_tol) < self.person_depth < (safe_distance + dist_tol) or self.person_depth < 0.1:
    #             self._linear = 0 # m/s
    #         elif self.person_depth > safe_distance + dist_tol:
    #             self._linear = 0.25
    #         elif self.person_depth < safe_distance - dist_tol:
    #             self._linear = -0.25
    #         # self._linear = 0
            
    #         diff_px = 320 - self.person_loc[0]
    #         if abs(diff_px) > rot_tol:
    #             self._angular = diff_px * (1/800)
    #         else:
    #             self._angular = 0.0
                
    #         twist = get_twist(self._linear, self._angular)
    #         self.pub_cmd.publish(twist)

 

def main():
    
    follower = PersonFollower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
