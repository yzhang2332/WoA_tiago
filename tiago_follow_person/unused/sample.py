#! /usr/bin/env python

#Only Open CV for the depth and RGB
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pal_detection_msgs.msg import Detections2d, Detection2d
import message_filters

class PersonDetector:
    def __init__(self):
        rospy.init_node('pal_person_detector_opencv', anonymous=True)

        self.bridge = CvBridge()
        # self.hog = cv2.HOGDescriptor()
        # self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Subscribers for RGB and Depth images
        self.rgb_sub = message_filters.Subscriber('/xtion/rgb/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/xtion/depth/image_raw', Image)

        # Synchronize the subscribers
        ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=5, slop=0.5)
        ts.registerCallback(self.callback)

        # self.detections_publisher = rospy.Publisher('~detections', Detections2d, queue_size=1)
        # self.debug_image_publisher = rospy.Publisher('~debug', Image, queue_size=1)


    def callback(self, rgb_data, depth_data):

        rospy.loginfo("Received RGB and Depth images")

        try:
            cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        cv2.imshow("RGB Image", cv_rgb_image)
        cv2.imshow("Depth Image", cv_depth_image)
        cv2.waitKey(1)

        # # Person detection on RGB image
        # (rects, _) = self.hog.detectMultiScale(cv_rgb_image, winStride=(4, 4), padding=(8, 8), scale=1.05)

        # detections_msg = Detections2d()
        # detections_msg.header = rgb_data.header

        # for (x, y, w, h) in rects:
        #     detection = Detection2d()
        #     detection.x = x
        #     detection.y = y
        #     detection.width = w
        #     detection.height = h

        #     # Calculate average depth in the bounding box
        #     depth_roi = cv_depth_image[y:y+h, x:x+w]
        #     non_zero_depths = depth_roi[depth_roi > 0]
        #     if len(non_zero_depths) > 0:
        #         average_depth = np.mean(non_zero_depths)
        #         detection.distance = average_depth  # Add the average depth to the detection message
        #     else:
        #         detection.distance = float('nan')  # No valid depth data

        #     detections_msg.detections.append(detection)
        #     cv2.rectangle(cv_rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # self.detections_publisher.publish(detections_msg)

        # try:
        #     self.debug_image_publisher.publish(self.bridge.cv2_to_imgmsg(cv_rgb_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)

if __name__ == '__main__':

    detector = PersonDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass



    # try:
    #     detector = PersonDetector()
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass



# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# import time 

# # 

# def tiago_control_head():

#     # initialize the ros node
#     rospy.init_node('tiago_head_controller') # node name
#     head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

#     head_traj = JointTrajectory()
#     head_traj.joint_names = ['head_1_joint', 'head_2_joint']

#     head_traj_point = JointTrajectoryPoint()
#     head_traj_point.positions = [0.4, 0.4] 
#     head_traj_point.time_from_start = rospy.Duration(1)

#     head_traj.points.append(head_traj_point)

#     rate = rospy.Rate(20)

#     while not rospy.is_shutdown():

#         head_pub.publish(head_traj)
#         rospy.loginfo('Message Published ...')

#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         tiago_control_head()
#     except rospy.ROSInterruptException:
#         pass