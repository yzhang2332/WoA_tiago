#!/usr/bin/env python
import rospy
import tf
import os

import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions  # For converting Euler angles to quaternion


# MAP_FRAME = '/map'
# ROBOT_BASE_FRAME = '/base_footprint'


##################################################
def get_transformation():

    (trans, rot) = listener.lookupTransform("base_footprint", "local_costmap", rospy.Time(0))
    print("Translation: ", trans)
    rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
    print("Rotation: ", rot_euler)
    new_pose_dict = {"translation": trans, "rotation": rot}
    return new_pose_dict


##################################################
if __name__ == '__main__':

    rospy.init_node('tf_listener', anonymous=True)

    listener = tf.TransformListener()
    rospy.sleep(1)
    print("Sleeping for 1 second ...")

    # frame_list = listener.getFrameStrings()
    # print("Available frames:", frame_list)
    while not rospy.is_shutdown():
        get_transformation()
        rospy.sleep(1.0)
    # print(pose_dict)

    # # write results to YAML
    # poi_name = str(input("Enter the Point-of-Interest name: "))
    
    # current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
    # yaml_file = os.path.join(current_dir, '..', 'config', 'poi.yaml')  # Navigate to the key_poses.yaml file

    # write_to_yaml(yaml_file, poi_name, pose_dict)
    # print("Transformation successfully written to YAML file!")
