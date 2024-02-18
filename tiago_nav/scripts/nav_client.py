#!/usr/bin/env python
import rospy
# import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

import os
import yaml


class NavigationClient:

    ##################################################
    def __init__(self):

        rospy.init_node('navigation_client_test')

        # Create a Publisher object
        self.nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        print("sleeping for 1 seconds")
        rospy.sleep(1)

        # load poi data
        self.poi_data = self.read_yaml_file()

        self.running = True


    ##################################################
    def read_yaml_file(self):
        current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
        yaml_file = os.path.join(current_dir, '..', 'config', 'poi.yaml')  # Navigate to the key_poses.yaml file
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file) or {}


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
        # pose_goal.header.stamp = rospy.Time.now()

        # # Set the position and orientation of the goal
        # pose_goal.pose.position.x = 0.628
        # pose_goal.pose.position.y = 0.462
        # pose_goal.pose.position.z = 0.0

        # Set the position and orientation of the goal
        pose_goal.pose.position.x = trans[0]
        pose_goal.pose.position.y = trans[1]
        pose_goal.pose.position.z = 0.0

        # Rotation: x, y, z, w
        # pose_goal.pose.orientation = Quaternion(0.0, 0.0, 0.9936, 0.113)
        # pose_goal.pose.orientation.x = 0.0
        # pose_goal.pose.orientation.y = 0.0
        # pose_goal.pose.orientation.z = 0.9936
        # pose_goal.pose.orientation.w = 0.113

        pose_goal.pose.orientation.x = rot[0]
        pose_goal.pose.orientation.y = rot[1]
        pose_goal.pose.orientation.z = rot[2]
        pose_goal.pose.orientation.w = rot[3]

        # Sends the goal to the action server
        self.nav_pub.publish(pose_goal)
        rospy.loginfo("Goal published")


    ##################################################
    def run(self):

        while self.running:
            poi_name = str(input("Enter the poi name: "))
            if poi_name == "q":
                break
            self.move_base_pub(poi_name)



# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        Nav = NavigationClient()
        Nav.run()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("done!")
