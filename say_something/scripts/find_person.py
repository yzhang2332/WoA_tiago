#! /usr/bin/env python

#Only Open CV for the depth and RGB
import rospy
# import cv2
# import numpy as np
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# from pal_detection_msgs.msg import Detections2d, Detection2d
# import message_filters

from detection_msgs.msg import BoundingBoxes
from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from time import sleep


class PersonTracker:
    
    def __init__(self):

        rospy.init_node('person_tracker', anonymous=True)

        # Bounding box subscriber
        self.yolo_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.callback)

        # Head tracking pose publisher
        self.tracking_point_pub = rospy.Publisher('/pixel_to_track', JointState, queue_size=1)


    def callback(self, data):

        # rospy.loginfo("Received bounding boxes!!")
        boxes = data.bounding_boxes
        
        for box in boxes:
            if box.Class == "person":
                print("Found the person!")
                x, y = compute_centre(box.xmin, box.ymin, box.xmax, box.ymax)
                msg = JointState()
                msg.position = [x, y, 1]
                self.tracking_point_pub.publish(msg)
                print("Published: x, y = %.3f, %.3f, sleeping for 1 seconds" % (x, y))
                # sleep(5)
            else:
                print("No person!")
                msg = JointState()
                msg.position = [0.0, 0.0, 0]
                self.tracking_point_pub.publish(msg)
                print("Published all zero values!")
                # sleep(5)


    
def compute_centre(xmin, ymin, xmax, ymax):
    x = (xmin + xmax) / 2
    y = (ymin + ymax) / 2
    return x, y
        

if __name__ == '__main__':

    tracker = PersonTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        # cv2.destroyAllWindows()
        pass
    except rospy.ROSInterruptException:
        pass
