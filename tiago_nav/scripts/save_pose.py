#!/usr/bin/env python
import rospy
import tf
import yaml
import os


MAP_FRAME = '/map'
ROBOT_BASE_FRAME = '/base_footprint'


##################################################
def get_transformation():
    (trans, rot) = listener.lookupTransform(MAP_FRAME, ROBOT_BASE_FRAME, rospy.Time(0))
    print("Translation: ", trans)
    print("Rotation: ", rot)
    new_pose_dict = {"translation": trans, "rotation": rot}
    return new_pose_dict

##################################################
def read_yaml_file(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file) or {}

##################################################
def write_to_yaml(yaml_file, poi_name, pose_dict):
    existing_data = read_yaml_file(yaml_file)
    data_dict = {"poi_name": poi_name, "pose": pose_dict}
    existing_data[poi_name] = data_dict
    with open(yaml_file, 'w') as file:
        yaml.dump(existing_data, file, default_flow_style=False)


##################################################
if __name__ == '__main__':

    rospy.init_node('lookup_tf_node')

    listener = tf.TransformListener()
    rospy.sleep(1)
    print("Sleeping for 1 second ...")

    # frame_list = listener.getFrameStrings()
    # print("Available frames:", frame_list)

    pose_dict = get_transformation()
    print(pose_dict)

    # write results to YAML
    poi_name = str(input("Enter the Point-of-Interest name: "))
    
    current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
    yaml_file = os.path.join(current_dir, '..', 'config', 'poi.yaml')  # Navigate to the key_poses.yaml file

    write_to_yaml(yaml_file, poi_name, pose_dict)
    print("Transformation successfully written to YAML file!")

