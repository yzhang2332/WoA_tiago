#!/usr/bin/env python
# import rospy
# import cv2
# import cv2.aruco as aruco
# import numpy as np
# import math
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# def rotationVectorToEulerAngles(rvec):
#     R, _ = cv2.Rodrigues(rvec)
#     sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
#     singular = sy < 1e-6
#     if not singular:
#         x = math.atan2(R[2,1] , R[2,2])
#         y = math.atan2(-R[2,0], sy)
#         z = math.atan2(R[1,0], R[0,0])
#     else:
#         x = math.atan2(-R[1,2], R[1,1])
#         y = math.atan2(-R[2,0], sy)
#         z = 0
#     return np.degrees(x), np.degrees(y), np.degrees(z)

# def detect_markers(frame, aruco_dict, parameters, cameraMatrix, distCoeffs):
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, rejectedCandidates = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
#     rvecs, tvecs, _objPoints = None, None, None
#     if ids is not None:
#         rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMatrix, distCoeffs)
#         for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
#             aruco.drawDetectedMarkers(frame, corners, ids)
#             euler_angles = rotationVectorToEulerAngles(rvec)
#             print(f"Marker ID: {ids[i][0]}")
#             X_avg = int((corners[i][0][0][0]+corners[i][0][1][0])/2)
#             Y_avg = int((corners[i][0][0][1]+corners[i][0][2][1])/2)
#             print(f"Location (x, y): ({X_avg}, {Y_avg})")
#             print(f"Euler Angles (Yaw): ({euler_angles[2]:.2f})")
#             cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, 0.03)
#     return frame, ids, rvecs, tvecs

# class ArucoDetector:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
#         self.cameraMatrix = np.array([[1000, 0, 320],
#                                       [0, 1000, 240],
#                                       [0, 0, 1]], dtype=float)
#         self.distCoeffs = np.zeros((4, 1))
#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#         self.parameters = cv2.aruco.DetectorParameters()

#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(e)
        
#         frame_marked, ids, rvecs, tvecs = detect_markers(cv_image, self.aruco_dict, self.parameters, self.cameraMatrix, self.distCoeffs)
#         # Here you might publish detection results or take action based on the detections

# if __name__ == '__main__':
#     rospy.init_node('tiago_aruco_detector', anonymous=True)
#     ad = ArucoDetector()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")



# import rospy
# import cv2
# import cv2.aruco as aruco
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image

# class ArucoDetector:
#     def __init__(self):
#         # Initialize the node
#         rospy.init_node('tiago_aruco_detector', anonymous=True)
        
#         # Create a CvBridge to convert ROS images to OpenCV format
#         self.bridge = CvBridge()

#         # Define the ArUco dictionary and parameters
#         self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#         self.parameters = cv2.aruco.DetectorParameters()
        
#         # Subscribe to the robot's camera image topic
#         self.image_sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)

#     def detect_markers(self, frame):
#         # Convert to grayscale
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         # Detect markers
#         corners, ids, rejectedCandidates = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

#         rospy.loginfo("corners: %s", corners)
#         rospy.loginfo("ids: %s", ids)
#         # Draw markers
#         frame_marked = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
#         return frame_marked, ids

#     def image_callback(self, data):
#         try:
#             # Convert the ROS image message to OpenCV format
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         # Detect ArUco markers
#         frame_marked, ids = self.detect_markers(cv_image)

#         # Display the frame with markers
#         cv2.imshow('Frame', frame_marked)
#         cv2.waitKey(3)

# if __name__ == '__main__':
#     ad = ArucoDetector()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
#     cv2.destroyAllWindows()




# def image_callback(msg):
#     bridge = CvBridge()
#     window_name = 'Tiago Camera View'
#     cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)  # Create a window that can be resized
#     cv2.resizeWindow(window_name, 1280, 1280) 
#     # Convert the ROS Image message to OpenCV2 format
#     cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

#     # Convert to grayscale
#     gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)

#     # Initialize the detector parameters using default values
#     aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
#     parameters = aruco.DetectorParameters()

#     # Detect the markers in the image
#     corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

#     if ids is not None:
#         # Draw detected markers
#         aruco.drawDetectedMarkers(cv2_img, corners, ids)
#         for i, corner in zip(ids, corners):
#             # Calculate marker center
#             center = corner[0].mean(axis=0)
#             # Display the ID near the marker
#             cv2.putText(cv2_img, str(i[0]), (int(center[0]), int(center[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
#             # print(f"Detected ArUco marker ID: {i[0]}")

#     # Display the resulting frame
#     cv2.imshow(window_name, cv2_img)
#     cv2.waitKey(3)

# def main():
#     rospy.init_node('tiago_aruco_detector')
#     image_topic = "/xtion/rgb/image_raw"  # Updated topic
#     rospy.Subscriber(image_topic, Image, image_callback)
#     rospy.spin()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import JointState

def rotationVectorToEulerAngles(rvec):
    R, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.degrees(x), np.degrees(y), np.degrees(z)



class ArucoDetector:
    def __init__(self):
        self.node_name = "aruco_detector"
        rospy.init_node(self.node_name, anonymous=True)
        self.bridge = CvBridge()
        # Change the topic to match the camera topic from your robot
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        self.cameraMatrix = np.array([[523.5531882248562, 0.0, 329.4655423070979],
                                       [0.0, 522.6007280941504, 227.58372039112177],
                                       [0.0, 0.0, 1.0]], dtype=float)
        
        # self.cameraMatrix = np.array([[1200, 0, 320],
        #                               [0, 1200, 240],
        #                               [0, 0, 1]], dtype=float)
        
        # self.distCoeffs = np.zeros((4, 1))  # Assuming no lens distortion
        self.distCoeffs = np.array([[0.034373339868345086, -0.12169690916340495, -0.007093351005146207, 0.002392767261606366, 0.0]], dtype=float)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        self.aruco_pub = rospy.Publisher("/aruco_pose", JointState, queue_size=1)
        self.aruco_published = False

        rospy.on_shutdown(self.shutdown_hook)


    def detect_markers(self, frame, aruco_dict, parameters, cameraMatrix, distCoeffs):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, distCoeffs)

            for i, corner in enumerate(corners):
                aruco.drawDetectedMarkers(frame, corners, ids)
                euler_angles = rotationVectorToEulerAngles(rvecs[i])
                print(f"Marker ID: {ids[i][0]}")
                X_avg = int((corners[i][0][0][0]+corners[i][0][1][0])/2)
                Y_avg = int((corners[i][0][0][1]+corners[i][0][2][1])/2)
                print(f"Location (x, y): ({X_avg}, {Y_avg})")
                #print(f"Marker ID: {ids[i][0]}, Euler Angles (Yaw): {euler_angles[2]:.2f}")
                x = tvecs[i][0][0]
                y = tvecs[i][0][1]
                z = tvecs[i][0][2]
                roll = euler_angles[0]
                pitch = euler_angles[1]
                yaw = euler_angles[2]
                print(f"Location (x, y, z): ({tvecs[i][0][0]:.4f}, {tvecs[i][0][1]:.4f}, {tvecs[i][0][2]:.4f})")
                print(X_avg, Y_avg)
                print(f"Euler Angles (Roll, Pitch, Yaw): {euler_angles[0]:.4f}, {euler_angles[1]:.4f}, {euler_angles[2]:.4f}")

                if not self.aruco_published:
                    # publish aruco pose
                    pose_msg = JointState()
                    pose_msg.position = [x, y, z, roll, pitch, yaw]
                    self.aruco_pub.publish(pose_msg)
                    print("Published aruco pose")
                    # self.aruco_published = True

                cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.03)
        return frame



    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        frame_marked = self.detect_markers(cv_image, self.aruco_dict, self.parameters, self.cameraMatrix, self.distCoeffs)
        cv2.imshow('Frame with ArUco Markers', frame_marked)
        cv2.waitKey(3)

    def shutdown_hook(self):
        rospy.loginfo(f"Shutting down {self.node_name}")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")