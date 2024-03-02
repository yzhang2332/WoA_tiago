
#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters
from yolov5 import YOLOv5  # Assuming using YOLOv5 and there exists a Python package for it

class YOLOPersonDetector:
    def __init__(self):
        rospy.init_node('yolo_person_detector', anonymous=True)

        self.bridge = CvBridge()
        self.yolo = YOLOv5("path_to_yolov5_model_weights.pt")  # Load YOLO model

        # Subscribers for RGB and Depth images
        self.rgb_sub = message_filters.Subscriber('/xtion/rgb/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/xtion/depth/image_raw', Image)

        # Synchronize the subscribers
        ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=5, slop=0.5)
        ts.registerCallback(self.callback)

    def callback(self, rgb_data, depth_data):
        try:
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Detect persons using YOLO
        detections = self.yolo.detect(cv_rgb_image)

        for detection in detections:
            if detection.class_name == "person":
                x, y, w, h = detection.bbox
                cv2.rectangle(cv_rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate average depth in the bounding box
                depth_roi = cv_depth_image[y:y+h, x:x+w]
                non_zero_depths = depth_roi[depth_roi > 0]
                if len(non_zero_depths) > 0:
                    average_depth = np.mean(non_zero_depths)
                    rospy.loginfo(f"Person detected at an average depth of: {average_depth} meters")
                else:
                    rospy.loginfo("Person detected but no valid depth data")

        cv2.imshow("Detection", cv_rgb_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    detector = YOLOPersonDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
