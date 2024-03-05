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
    def __init__(self):

        rospy.init_node('navigation_function', anonymous=True)

        # # Subscribe to /mobile_base_controller/cmd_vel
        # self.vel_sub = rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, self.vel_callback)

        self.listener = tf.TransformListener()

        # Create a Publisher object
        self.nav_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        print("sleeping for 1 seconds")
        rospy.sleep(1)

        # load poi data
        self.poi_data = self.read_yaml_file()

        # # ignore messages within the first second from the cmd_vel topic (they might be all zeros)
        # # publishing frequency of cmd_vel = 10 Hz
        # self.msg_count = 0
        # self.msg_count_thres = 10

        self.map_frame = '/map'
        self.robot_base_frame = '/base_footprint'


    ##################################################
    def read_yaml_file(self):
        current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
        yaml_file = os.path.join(current_dir, '..', 'config', 'poi.yaml')  # Navigate to the key_poses.yaml file
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file) or {}


    ##################################################
    def vel_callback(self, msg):
        # print("received message")
        self.msg_count += 1
        if self.msg_count > 10:
            self.reached_goal = self.vel_is_zero(msg)


    ##################################################
    def vel_is_zero(self, vel_msg: Twist) -> bool:
        lin = vel_msg.linear
        ang = vel_msg.angular
        vels_list = [lin.x, lin.y, lin.z, ang.x, ang.y, ang.z]
        for vel in vels_list:
            if vel > 0.0:
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

        counter = 0

        reached_goal = False

        while not reached_goal and counter < 10:
            counter += 1
            rospy.loginfo("Waiting for Tiago to reach the Navigation goal ...")
            (curr_trans, curr_rot) = self.listener.lookupTransform(self.map_frame, self.robot_base_frame, rospy.Time(0))
            print(trans)
            # same_trans = self.are_same_3vec(trans, curr_trans)
            # same_rot = self.are_same_rot(rot, curr_rot)
            # print("Same translation = %s, Same rotation = %s" % (same_trans, same_rot))
            rospy.sleep(0.5)
        
        rospy.loginfo("Reached goal!!!")


    def are_same_rot(self, q1, q2):
        e1 = euler_from_quaternion(q1)
        e2 = euler_from_quaternion(q2)
        same_rot = self.are_same_3vec(e1, e2)
        return same_rot


    def are_same_3vec(self, t1, t2):
        if abs(t1[0]-t2[0]) < 10.0 and abs(t1[1]-t2[1]) < 10.0 and abs(t1[2]-t2[2]) < 10.0:
            return True
        return False


    ##################################################
    def run(self, poi_name):

        # nav goal reached flag
        self.reached_goal = False

        # send goal
        self.move_base_pub(poi_name)
            

        # self.msg_count = 0



# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        Nav = NavigationClient()
        # rospy.spin()
        # Nav.run()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("done!")
