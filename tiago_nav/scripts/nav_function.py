#!/usr/bin/env python
import rospy
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import os
import yaml
import tf
from tf.transformations import euler_from_quaternion


class NavigationClient:

    ##################################################
    def __init__(self, init_poi):
        # Subscribe to /mobile_base_controller/cmd_vel
        # self.vel_sub = rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, self.vel_callback)
        self.vel_sub = rospy.Subscriber('/nav_vel', Twist, self.vel_callback)

        # Create a Publisher object
        self.nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        print("sleeping for 1 seconds")
        rospy.sleep(1)

        # load poi data
        self.poi_data = self.read_yaml_file()

        # ignore messages within the first second from the cmd_vel topic (they might be all zeros)
        # publishing frequency of cmd_vel = 10 Hz
        self.msg_count = 0
        self.msg_thres = 10

        self.last_zero = False
        self.this_zero = False

        # store the current poi
        self.current_poi = init_poi


    ##################################################
    def read_yaml_file(self):
        current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
        yaml_file = os.path.join(current_dir, '..', 'config', 'poi.yaml')  # Navigate to the key_poses.yaml file
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file) or {}


    ##################################################
    def vel_callback(self, msg):
        # print("received message")
        self.reached_goal = False
        self.msg_count += 1
        
        if self.vel_is_zero(msg):
            print("Msg #%d is zero!" % self.msg_count)

        if self.msg_count > self.msg_thres:
            # have ignored the first 10 messages, start checking for 2 consecutive zero Twist messages now
            self.this_zero = self.vel_is_zero(msg)
            if self.last_zero or self.this_zero:
                self.reached_goal = True
            else:
                self.last_zero = self.this_zero


    ##################################################
    def vel_is_zero(self, vel_msg: Twist) -> bool:
        lin = vel_msg.linear
        ang = vel_msg.angular
        vels_list = [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
        for vel in vels_list:
            if abs(vel) > 0.0:
                return False
        return True


    ##################################################
    def move_base_pub(self, poi_name):
        
        # load pose for the given poi
        poi_dict = self.poi_data[poi_name]
        desired_pose = poi_dict['pose']
        trans = desired_pose['translation']
        rot = desired_pose['rotation']

        print("Translation: %s" % trans)
        print("Rotation: %s" % rot)

        # Create a PoseStamped goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "map"
        pose_goal.header.stamp = rospy.Time.now()

        # Set the position and orientation of the goal
        pose_goal.pose.position.x = trans[0]
        pose_goal.pose.position.y = trans[1]
        pose_goal.pose.position.z = 0.0

        pose_goal.pose.orientation.x = rot[0]
        pose_goal.pose.orientation.y = rot[1]
        pose_goal.pose.orientation.z = rot[2]
        pose_goal.pose.orientation.w = rot[3]

        # Sends the goal to the action server
        self.nav_pub.publish(pose_goal)
        rospy.loginfo("Goal published")        


    ##################################################
    def run(self, poi_name):

        self.msg_count = 0

        # send goal (only if not already at it)
        if self.current_poi != poi_name:
            
            self.reached_goal = False
            self.move_base_pub(poi_name)

            while not self.reached_goal:
                # rospy.loginfo("Waiting for Tiago to reach the Navigation goal ...")
                rospy.sleep(0.1)
        
            rospy.loginfo("Reached goal!!!")
            self.current_poi = poi_name

        else:

            rospy.loginfo("Already at goal!")

        
        print("Navigation function finished!")

        



# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('navigation_function', anonymous=True)
        Nav = NavigationClient("one")
        # rospy.spin()
        # Nav.run()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("done!")
