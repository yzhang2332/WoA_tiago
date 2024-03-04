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


class ObstacleChecker:

    def __init__(self):

        rospy.init_node('obstacle_checker', anonymous=True)
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.callback)

        self.obstacle_dir_pub = rospy.Publisher('/obstacle_dir', JointState, queue_size=1)

        self.listener = tf.TransformListener()
        rospy.sleep(1)
        print("Sleeping for 1 second ...")

    def run(self):
        rospy.spin()

    
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
        return rot_euler
    

    def find_directions(self, map: np.ndarray) -> list:
        
        directions = [0, 0, 0, 0]
        length = map.shape[0]
        width = int(length / 4)

        # anti-clockwise definition:
        # 1 - front, 2 - left, 3 - back, 4 - right

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
    

    def callback(self, data):

        # rospy.loginfo("Received local costmap data")

        # Example: Check a specific cell
        # x, y = 1.0, 1.0  # coordinatesn to check (in the robot's frame), in m
        # x, y = 0.79, 0.79  # coordinatesn to check (in the robot's frame), in m

        # resolution = data.info.resolution
        resolution = 0.025    # m / px
        width = data.info.width
        height = data.info.height
        origin = data.info.origin

        origin_position = origin.position
        q = origin.orientation
        # print(q)
        rot_euler = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        # print(rot_euler)

        # publish transformation of {local_costmap} in {odom}
        self.publish_static_transformation('odom', 'local_costmap', origin_position, q)
        # print("Published static transform!")

        # print("resolution = %.10f" % resolution)
        # print("height = %.3f" % height)
        # print("width = %.3f" % width)
        # print("origin position: x = %.3f, y = %.3f" % (origin_position.x, origin_position.y))
        # print("origin orientation: x = %.3f, y = %.3f" % (origin_orientation.x, origin_orientation.y))

        center_px = 79

        margin = 0.8   # in m
        margin_px = round(margin / resolution)   # in pixels of the matrix
        window_width = margin_px * 2
        # print("margin_px = %d, window_width = %d" % (margin_px, window_width))

        values = []

        for grid_x in range(center_px - margin_px, center_px + margin_px):
            for grid_y in range(center_px - margin_px, center_px + margin_px):
                index = grid_y * width + grid_x
                # Check if the cell is occupied
                if 0 <= index < len(data.data):
                    values.append(data.data[index])
                #     if data.data[index] == 100:
                #         rospy.loginfo("Obstacle detected at grid (%d, %d)", grid_x, grid_y)
                #     else:
                #         rospy.loginfo("No obstacle at grid (%d, %d)", grid_x, grid_y)
                # else:
                #     rospy.loginfo("Coordinates out of bounds")

        # Convert the tuple to a NumPy array (also convert all 100 -> 1)
        values_np = np.array(values, dtype=np.uint8) / 100

        # Reshape the array to 2D - 80 rows, 80 columns
        values_mat = values_np.reshape(window_width, window_width)

        # count_before = np.count_nonzero(values_mat == 1)
        # print("count_before = %d" % count_before)

        # ORIGINAL -> EROSION -> DILATION
        
        # Taking a matrix of size 5 as the kernel 
        kernel_size = 4
        kernel = np.ones((kernel_size, kernel_size), np.uint8) 
        # threshold = floor(kernel_size**2 / 3)
        threshold = 10
        # print(threshold)
        
        # The first parameter is the original image, 
        # kernel is the matrix with which image is 
        # convolved and third parameter is the number 
        # of iterations, which will determine how much 
        # you want to erode/dilate a given image. 

        # img_erosion = cv2.erode(values_mat, kernel, iterations=1) 
        img_erosion = erode(values_mat, kernel, threshold)
        
        img_dilation = cv2.dilate(img_erosion, kernel, iterations=1) 

        #print(img_dilation)


        # rotate image
        rot_euler = self.get_transformation()
        # print(rot_euler)
        yaw_deg = rot_euler[2]/pi*180 - 180
        # print(yaw_deg)

        rotated_img = rotate(img_dilation, yaw_deg, reshape=False, mode='reflect')
        rotated_img = np.round(rotated_img)

        # publish the directions vector
        directions = self.find_directions(rotated_img)
        dir_msg = JointState()
        dir_msg.position = directions
        self.obstacle_dir_pub.publish(dir_msg)


        # cv2.imshow('Dilation', img_dilation2) 
        # cv2.imshow('Dilation', rotated_img) 
        # cv2.waitKey(0) 


        # count_after = np.count_nonzero(img_dilation == 1)
        # count_after = np.count_nonzero(rotated_img == 1)
        # print("count_after = %d" % count_after)

        # if count_after > 0:
        #     # obstacle detected
        #     # print("OBSTACLE DETECTED!!!")
        #     col_flag = JointState()
        #     col_flag.data = True
        #     self.obstacle_flag_pub.publish(col_flag)

        #     directions = self.find_directions(rotated_img)
        #     self.recover_from_obstacle(directions)

        # else:
        #     # all clear
        #     # print("ALL CLEAR!!!!")
        #     col_flag = Bool()
        #     col_flag.data = False
        #     self.obstacle_flag_pub.publish(col_flag)

        
        # print(img_dilation2.shape)

        # cv2.imshow('Input', values_mat) 
        # cv2.imshow('Erosion', img_erosion) 
        

        # # Convert coordinates to cell index
        # grid_x = int((x - origin.x) / resolution)
        # grid_y = int((y - origin.y) / resolution)

        # grid_x = int(x / resolution)
        # grid_y = int(y / resolution)

        # grid_x = 159
        # grid_y = 159
        # index = grid_y * width + grid_x

        # # Check if the cell is occupied
        # if 0 <= index < len(data.data):
        #     if data.data[index] == 100:
        #         rospy.loginfo("Obstacle detected at grid (%d, %d)", grid_x, grid_y)
        #     else:
        #         rospy.loginfo("No obstacle at grid (%d, %d)", grid_x, grid_y)
        # else:
        #     rospy.loginfo("Coordinates out of bounds")



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
