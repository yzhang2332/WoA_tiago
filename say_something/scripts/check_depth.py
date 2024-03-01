#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class DepthImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/xtion/depth/image_raw", Image, self.depth_callback)
        

    def depth_callback(self, data):
        try:
            # Convert the depth image using the default passthrough encoding
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            
            # Convert to a Numpy array for easier manipulation
            depth_array = np.array(cv_image, dtype=np.float32)
            # Mask the zeros
            masked_array = np.ma.masked_equal(depth_array, 0)

            # Find the minimum non-zero value
            min_non_zero = masked_array.max()
            # min_index = np.where(masked_array == min_non_zero)

            print("Minimum non-zero value:", min_non_zero)
            # print("Index of minimum non-zero value:", min_index)


            # The depth image is a single-channel float32 image
            # The values are in millimeters
            # You can now process this array to get depth values at specific pixels

            # Example: Get the depth value at the center of the image
            # center_idx = (depth_array.shape[0] // 2, depth_array.shape[1] // 2)
            # depth = depth_array[center_idx[0], center_idx[1]]
            # print(f"Depth at center: {depth} mm")

        except CvBridgeError as e:
            print(e)

def main():
    rospy.init_node('depth_image_processor', anonymous=True)
    dip = DepthImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
