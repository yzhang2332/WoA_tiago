#!/usr/bin/env python
import rospy
import tf
import os

import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions  # For converting Euler angles to quaternion


# MAP_FRAME = '/map'
# ROBOT_BASE_FRAME = '/base_footprint'


def publish_static_transformation(parent_frame_id, child_frame_id, translation, rotation):
    """
    Publish a static transformation between two frames to tf in ROS.

    Args:
    - parent_frame_id (str): The parent frame ID.
    - child_frame_id (str): The child frame ID.
    - translation (tuple): A tuple of 3 elements (x, y, z) representing the translation.
    - rotation (tuple): A tuple of 3 elements (roll, pitch, yaw) representing the rotation in Euler angles.

    This function publishes a static transformation that can be listened to by other ROS nodes.
    """

    # Create a StaticTransformBroadcaster object
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a TransformStamped message
    static_transform_stamped = TransformStamped()
    
    # Fill the message with the necessary data
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = parent_frame_id
    static_transform_stamped.child_frame_id = child_frame_id
    static_transform_stamped.transform.translation.x = translation[0]
    static_transform_stamped.transform.translation.y = translation[1]
    static_transform_stamped.transform.translation.z = translation[2]

    # Convert Euler angles to a quaternion
    q = tf_conversions.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
    
    static_transform_stamped.transform.rotation.x = q[0]
    static_transform_stamped.transform.rotation.y = q[1]
    static_transform_stamped.transform.rotation.z = q[2]
    static_transform_stamped.transform.rotation.w = q[3]

    # Send the transformation
    static_broadcaster.sendTransform(static_transform_stamped)
    rospy.loginfo("Static transform published from {} to {}.".format(parent_frame_id, child_frame_id))



##################################################
if __name__ == '__main__':

    rospy.init_node('tf_broadcaster', anonymous=True)

    listener = tf.TransformListener()
    rospy.sleep(1)
    print("Sleeping for 1 second ...")

    # frame_list = listener.getFrameStrings()
    # print("Available frames:", frame_list)

    publish_static_transformation('odom', 'local_costmap', (1.0, 2.0, 3.0), (0.0, 0.0, 1.57))
    print("Published static transform")
    rospy.sleep(100.0)

    # while not rospy.is_shutdown():
    #     print("Publishing transformation!")
        
        # rospy.sleep(0.01)

    # print(pose_dict)

    # # write results to YAML
    # poi_name = str(input("Enter the Point-of-Interest name: "))
    
    # current_dir = os.path.dirname(__file__)  # Gets the directory of the current script
    # yaml_file = os.path.join(current_dir, '..', 'config', 'poi.yaml')  # Navigate to the key_poses.yaml file

    # write_to_yaml(yaml_file, poi_name, pose_dict)
    # print("Transformation successfully written to YAML file!")
