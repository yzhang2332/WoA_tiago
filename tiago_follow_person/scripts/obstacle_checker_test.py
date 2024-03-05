#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from math import floor, pi
# from tf.transformations import euler_from_quaternion
import tf_conversions
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.ndimage import rotate
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header


class ObstacleChecker:

    def __init__(self):

        rospy.init_node('obstacle_checker', anonymous=True)

        self.recovering = False
        self.check_col_dist = 1.0   # meters

        self.received_initial_costmap = False

        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.costmap_callback)
        rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGrid, self.costmap_update_callback)

        # rospy.Subscriber("/move_base/local_costmap/costmap_updates", OccupancyGrid, self.callback)
        rospy.Subscriber("/recovering", Bool, self.recover_callback)

        self.filtered_map_pub = rospy.Publisher("/filtered_local_costmap", OccupancyGrid, queue_size=1)

        self.obstacle_dir_pub = rospy.Publisher('/obstacle_dir', JointState, queue_size=1)

        self.listener = tf.TransformListener()
        rospy.sleep(1)
        print("Sleeping for 1 second ...")



    def run(self):
        rospy.spin()


    def recover_callback(self, msg: Bool):
        self.recovering = msg.data

    
    def publish_static_transformation(self, parent_frame_id, child_frame_id, translation, rotation):

        # Create a StaticTransformBroadcaster object
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()
        
        # Fill the message with the necessary data
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = parent_frame_id
        static_transform_stamped.child_frame_id = child_frame_id
        static_transform_stamped.transform.translation.x = translation.x
        static_transform_stamped.transform.translation.y = translation.y
        static_transform_stamped.transform.translation.z = translation.z
        
        static_transform_stamped.transform.rotation.x = rotation.x
        static_transform_stamped.transform.rotation.y = rotation.y
        static_transform_stamped.transform.rotation.z = rotation.z
        static_transform_stamped.transform.rotation.w = rotation.w

        # Send the transformation
        static_broadcaster.sendTransform(static_transform_stamped)
        rospy.loginfo("Static transform published from {} to {}.".format(parent_frame_id, child_frame_id))
        rospy.loginfo("Sleeping for 2 seconds")
        rospy.sleep(0.5)


    def get_transformation(self):

        (trans, rot) = self.listener.lookupTransform("base_footprint", "local_costmap", rospy.Time(0))
        # print("Translation: ", trans)
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        # print("Rotation: ", rot_euler)
        # new_pose_dict = {"translation": trans, "rotation": rot}
        # return new_pose_dict
        return trans, rot, rot_euler
    

    def find_directions(self, map: np.ndarray) -> list:
        
        directions = [0, 0, 0, 0]
        length = map.shape[0]
        width = int(length / 4)

        # anti-clockwise definition:
        # indices of the list: 0 - front, 1 - left, 2 - back, 3 - right

        # check front
        front_arr = map[0:width, 0:length]
        if np.count_nonzero(front_arr == 1):
            directions[0] = 1

        # check left
        left_arr = map[0:length, 0:width]
        if np.count_nonzero(left_arr == 1):
            directions[1] = 1

        # check back
        back_arr = map[length-width:length, 0:length]
        if np.count_nonzero(back_arr == 1):
            directions[2] = 1

        # check right
        right_arr = map[0:length, length-width:length]
        if np.count_nonzero(right_arr == 1):
            directions[3] = 1

        return directions


    def numpy_to_occupancy_grid_msg(self, array, position, rot, resolution, frame_id):
        """
        Converts a numpy 2D array into a nav_msgs/OccupancyGrid message.

        Parameters:
        - array: 2D numpy array representing the occupancy data.
                Values should be in the range [0, 100] for occupied probabilities,
                -1 for unknown.
        - resolution: The resolution of the grid, in meters/cell.
        - frame_id: The id of the reference coordinate frame for the grid.

        Returns:
        - occupancy_grid_msg: The OccupancyGrid message.
        """
        # Ensure the numpy array is of type 'int8'
        array = array.astype(np.int8)

        # Create an OccupancyGrid message
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = rospy.Time.now()
        occupancy_grid_msg.header.frame_id = frame_id

        occupancy_grid_msg.info.resolution = resolution
        occupancy_grid_msg.info.width = array.shape[1]
        occupancy_grid_msg.info.height = array.shape[0]
        # occupancy_grid_msg.info.origin.position.x = position.x
        # occupancy_grid_msg.info.origin.position.y = position.y
        # occupancy_grid_msg.info.origin.position.z = position.z
        occupancy_grid_msg.info.origin.position.x = position[0]
        occupancy_grid_msg.info.origin.position.y = position[1]
        occupancy_grid_msg.info.origin.position.z = position[2]

        quat = tf_conversions.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
        # occupancy_grid_msg.info.origin.orientation.x = orientation.x
        # occupancy_grid_msg.info.origin.orientation.y = orientation.y
        # occupancy_grid_msg.info.origin.orientation.z = orientation.z
        # occupancy_grid_msg.info.origin.orientation.w = orientation.w
        occupancy_grid_msg.info.origin.orientation.x = quat[0]
        occupancy_grid_msg.info.origin.orientation.y = quat[1]
        occupancy_grid_msg.info.origin.orientation.z = quat[2]
        occupancy_grid_msg.info.origin.orientation.w = quat[3]

        # Flatten the array and convert it to a list
        occupancy_grid_msg.data = (array*100).ravel().tolist()

        return occupancy_grid_msg
    

    ###############################################################
    def costmap_update_callback(self, data):

        # resolution = data.info.resolution
        # resolution = 0.025    # m / px
        # width = data.info.width
        # height = data.info.height
        # origin = data.info.origin

        # origin_position = origin.position
        # q = origin.orientation
        # # print(q)
        # rot_euler = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # print(rot_euler)

        # publish transformation of {local_costmap} in {odom}
        self.publish_static_transformation('odom', 'local_costmap', self.origin_position, self.q)
        # print("Published static transform!")

        center_px = 79

        margin = self.check_col_dist             # in m
        margin_px = round(margin / self.resolution)   # in pixels of the matrix
        window_width = margin_px * 2
        # print("margin_px = %d, window_width = %d" % (margin_px, window_width))

        values = []

        for grid_x in range(center_px - margin_px, center_px + margin_px):
            for grid_y in range(center_px - margin_px, center_px + margin_px):
                index = grid_y * self.width + grid_x
                # Check if the cell is occupied
                if 0 <= index < len(data.data):
                    values.append(data.data[index])

        # Convert the tuple to a NumPy array (also convert all 100 -> 1)
        values_np = np.array(values, dtype=np.uint8) / 100

        # Reshape the array to 2D - 80 rows, 80 columns
        values_mat = values_np.reshape(window_width, window_width)

        # ORIGINAL -> EROSION -> DILATION
        
        # Taking a matrix of size 5 as the kernel 
        kernel_size = 4
        kernel = np.ones((kernel_size, kernel_size), np.uint8) 
        # threshold = floor(kernel_size**2 / 3)
        threshold = 10
        # img_erosion = cv2.erode(values_mat, kernel, iterations=1) 
        img_erosion = erode(values_mat, kernel, threshold)
        
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=1) 

        #print(img_dilation)

        ###### rotate image ######
        ###### only for computing where the obstacles are relative to the robot ######
        trans, rot, rot_euler = self.get_transformation()
        # print(rot_euler)
        yaw_deg = rot_euler[2]/pi*180 - 180
        # print(yaw_deg)

        rotated_img = rotate(img_dilation, yaw_deg, reshape=False, mode='reflect')
        rotated_img = np.round(rotated_img)

        ###### Publish the map to display in RViz ######
        filtered_map_trans = [margin, margin, 0.0]
        filtered_map_rot = [0.0, pi, pi/2]
        filtered_map_msg = self.numpy_to_occupancy_grid_msg(rotated_img, filtered_map_trans, filtered_map_rot, self.resolution, "base_footprint")
        self.filtered_map_pub.publish(filtered_map_msg)
        rospy.loginfo("Published filtered costmap")

        # publish the directions vector
        directions = self.find_directions(rotated_img)
        dir_msg = JointState()
        dir_msg.position = directions
        self.obstacle_dir_pub.publish(dir_msg)


    ###############################################################
    def costmap_callback(self, data):

        if self.received_initial_costmap:
            return
        
        self.resolution = 0.025    # m / px
        self.width = data.info.width
        self.height = data.info.height
        self.origin = data.info.origin

        self.origin_position = self.origin.position
        self.q = self.origin.orientation
        # print(q)
        self.rot_euler = tf_conversions.transformations.euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])

        # resolution = data.info.resolution
        # resolution = 0.025    # m / px
        # width = data.info.width
        # height = data.info.height
        # origin = data.info.origin

        # origin_position = origin.position
        # q = origin.orientation
        # # print(q)
        # rot_euler = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # print(rot_euler)

        # publish transformation of {local_costmap} in {odom}
        self.publish_static_transformation('odom', 'local_costmap', self.origin_position, self.q)
        # print("Published static transform!")

        center_px = 79

        margin = self.check_col_dist             # in m
        margin_px = round(margin / self.resolution)   # in pixels of the matrix
        window_width = margin_px * 2
        # print("margin_px = %d, window_width = %d" % (margin_px, window_width))

        values = []

        for grid_x in range(center_px - margin_px, center_px + margin_px):
            for grid_y in range(center_px - margin_px, center_px + margin_px):
                index = grid_y * self.width + grid_x
                # Check if the cell is occupied
                if 0 <= index < len(data.data):
                    values.append(data.data[index])

        # Convert the tuple to a NumPy array (also convert all 100 -> 1)
        values_np = np.array(values, dtype=np.uint8) / 100

        # Reshape the array to 2D - 80 rows, 80 columns
        values_mat = values_np.reshape(window_width, window_width)

        # ORIGINAL -> EROSION -> DILATION
        
        # Taking a matrix of size 5 as the kernel 
        kernel_size = 4
        kernel = np.ones((kernel_size, kernel_size), np.uint8) 
        # threshold = floor(kernel_size**2 / 3)
        threshold = 10
        # img_erosion = cv2.erode(values_mat, kernel, iterations=1) 
        img_erosion = erode(values_mat, kernel, threshold)
        
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=1) 

        #print(img_dilation)

        ###### rotate image ######
        ###### only for computing where the obstacles are relative to the robot ######
        trans, rot, rot_euler = self.get_transformation()
        # print(rot_euler)
        yaw_deg = rot_euler[2]/pi*180 - 180
        # print(yaw_deg)

        rotated_img = rotate(img_dilation, yaw_deg, reshape=False, mode='reflect')
        rotated_img = np.round(rotated_img)

        ###### Publish the map to display in RViz ######
        filtered_map_trans = [margin, margin, 0.0]
        filtered_map_rot = [0.0, pi, pi/2]
        filtered_map_msg = self.numpy_to_occupancy_grid_msg(rotated_img, filtered_map_trans, filtered_map_rot, self.resolution, "base_footprint")
        self.filtered_map_pub.publish(filtered_map_msg)
        rospy.loginfo("Published filtered costmap")

        # publish the directions vector
        directions = self.find_directions(rotated_img)
        dir_msg = JointState()
        dir_msg.position = directions
        self.obstacle_dir_pub.publish(dir_msg)

        self.received_initial_costmap = True



def erode(image: np.ndarray, kernel: np.ndarray, threshold):

    kernel_height, kernel_width = kernel.shape
    image_height, image_width = image.shape

    # Padding size
    pad_height = kernel_height // 2
    pad_width = kernel_width // 2

    # Pad the image with zeros on the border
    padded_image = np.pad(image, ((pad_height, pad_height), (pad_width, pad_width)), mode='constant', constant_values=0)
    
    # Initialize the eroded image
    eroded_image = np.zeros_like(image)

    # Perform erosion
    for i in range(image_height):
        for j in range(image_width):
            # Extract the region of interest
            region = padded_image[i:i+kernel_height, j:j+kernel_width]
            # Erode if all values in the kernel match corresponding values in the region
            equal_count = np.sum(region == kernel)

            if equal_count > threshold:
                eroded_image[i, j] = 1
            else:
                eroded_image[i, j] = 0


    return eroded_image



 
def main():

    # rospy.init_node('local_costmap_listener', anonymous=True)
    checker = ObstacleChecker()
    checker.run()

    # rospy.spin()



if __name__ == '__main__':
    main()
