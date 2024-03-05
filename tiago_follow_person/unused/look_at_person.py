#!/usr/bin/env python

import rospy
import actionlib
import cv2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal

import numpy as np
from sensor_msgs.msg import JointState

# Constants
WINDOW_NAME = "Inside of TIAGo's head"
CAMERA_FRAME = "/xtion_rgb_optical_frame"
IMAGE_TOPIC = "/xtion/rgb/image_raw"
CAMERA_INFO_TOPIC = "/xtion/rgb/camera_info"
PIXEL_TOPIC = "/pixel_to_track"

# Global Variables
camera_intrinsics = None
latest_image_stamp = None
point_head_client = None
pixelx = 0
pixely = 0


def image_callback(img_msg):

    global latest_image_stamp
    latest_image_stamp = img_msg.header.stamp

    # bridge = cv_bridge.CvBridge()
    # cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    # cv2.imshow(WINDOW_NAME, cv_image)
    # cv2.waitKey(15)


def create_point_head_client():
    global point_head_client
    point_head_client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
    point_head_client.wait_for_server(rospy.Duration(5))
    print("Connect head server.")


def look_to_point(u, v):
    global camera_intrinsics, latest_image_stamp

    if camera_intrinsics is None:
        rospy.logerr("Camera intrinsics not yet available.")
        return

    point_stamped = PointStamped()
    point_stamped.header.frame_id = CAMERA_FRAME
    point_stamped.header.stamp = latest_image_stamp

    x = (u - camera_intrinsics[0, 2]) / camera_intrinsics[0, 0]
    y = (v - camera_intrinsics[1, 2]) / camera_intrinsics[1, 1]
    Z = 1.0  # Define an arbitrary distance

    point_stamped.point.x = x * Z
    point_stamped.point.y = y * Z
    point_stamped.point.z = Z

    goal = PointHeadGoal()
    goal.pointing_frame = CAMERA_FRAME
    goal.pointing_axis.x = 0.0
    goal.pointing_axis.y = 0.0
    goal.pointing_axis.z = 1.0
    goal.min_duration = rospy.Duration(1.0)
    goal.max_velocity = 0.3
    goal.target = point_stamped

    print("Sending goal: u = %d, v = %d" % (u, v))
    point_head_client.send_goal(goal)
    point_head_client.wait_for_result(rospy.Duration(5))


def camera_info_callback(camera_info_msg):
    global camera_intrinsics
    camera_intrinsics = np.zeros((3, 3), dtype=np.float64)
    camera_intrinsics[0, 0] = camera_info_msg.K[0]  # fx
    camera_intrinsics[1, 1] = camera_info_msg.K[4]  # fy
    camera_intrinsics[0, 2] = camera_info_msg.K[2]  # cx
    camera_intrinsics[1, 2] = camera_info_msg.K[5]  # cy
    camera_intrinsics[2, 2] = 1


def pixel_callback(msg):
    x = msg.position[0]
    y = msg.position[1]
    # global pixelx, pixely
    # print("Received pixel: x = %.3f, y = %.3f!" % (x, y))
    # pixelx = int(x)
    # pixely = int(y)
    ix = int(x)
    iy = int(y)
    # print("Sending goal: u = %d, v = %d" % (ix, iy))
    look_to_point(ix, iy)


def main():

    rospy.init_node('look_at_person')

    rospy.wait_for_message(CAMERA_INFO_TOPIC, CameraInfo, timeout=rospy.Duration(10))

    rospy.Subscriber(CAMERA_INFO_TOPIC, CameraInfo, camera_info_callback)

    rospy.Subscriber(PIXEL_TOPIC, JointState, pixel_callback)

    create_point_head_client()

    # cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    rospy.Subscriber(IMAGE_TOPIC, Image, image_callback)

    rospy.spin()

    # while not rospy.is_shutdown():
    #     image_callback()
    #     pixel_callback()
    #     # Replace this with input method of choice
    #     # u, v = map(int, input("Enter pixel coordinates (u, v): ").split())
    #     look_to_point(pixelx, pixely)
    #     pass

    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
