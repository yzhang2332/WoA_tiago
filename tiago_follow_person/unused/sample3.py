#!/usr/bin/env python

#Only RGB

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class RGBImageViewer:
    def __init__(self):
        rospy.init_node('rgb_image_viewer', anonymous=True)

        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber('/xtion/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        

        cv2.imshow("RGB Image", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    viewer = RGBImageViewer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass
