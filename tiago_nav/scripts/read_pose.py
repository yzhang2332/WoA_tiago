#!/usr/bin/env python
import yaml
import os


##################################################
def read_yaml_file(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file) or {}
    

##################################################
if __name__ == '__main__':
    
    current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
    yaml_file = os.path.join(current_dir, '..', 'config', 'poi.yaml')  # Navigate to the key_poses.yaml file

    data = read_yaml_file(yaml_file)

    poi_dict = data['one']
    this_pose = poi_dict['pose']
    trans = this_pose['translation']
    rot = this_pose['rotation']

    print(trans)
    print(rot)