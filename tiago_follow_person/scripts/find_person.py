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


###### CLASS DEFINITION ######
class PersonFinder:
    
    def __init__(self):

        rospy.init_node('person_finder', anonymous=True)

        # Bounding box subscriber (yolov5)
        # self.yolov5_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.callback)

        # Bounding box subscriber (yolov8)
        self.yolov8_sub = rospy.Subscriber('/yolov8/detections', BoundingBoxes, self.callback)

        # Head tracking pose publisher
        self.tracking_point_pub = rospy.Publisher('/pixel_to_track', JointState, queue_size=1)


    def callback(self, data:BoundingBoxes):

        boxes = data.bounding_boxes

        num_person = 0
        centers = []
        areas = []
        
        for box in boxes:
            if box.Class == "person":
                # print("Found a person!")
                num_person += 1
                # compute the center and area of the bounding boxes
                x, y = self.compute_centre(box.xmin, box.ymin, box.xmax, box.ymax)
                area = self.compute_area(box.xmin, box.ymin, box.xmax, box.ymax)
                # add them to the list of bounding boxes containing people
                centers.append((x, y))
                areas.append(area)
        
        # self.print_info(num_person, centers, areas)

        # Publish pixel to track ONLY IF person is non zero
        if num_person > 0:
            target = self.get_max_bb_center(centers, areas)
            self.send_pixel_to_track(target[0], target[1])
        else:
            self.send_emtpy_msg()


    ###### message sender functions ######
    def send_pixel_to_track(self, x, y) -> None:
        msg = JointState()
        msg.position = [x, y, 1]
        self.tracking_point_pub.publish(msg)
        # rospy.loginfo("Sending pixel to track: x, y = %.3f, %.3f" % (x, y))
    
    def send_emtpy_msg(self) -> None:
        msg = JointState()
        msg.position = [0.0, 0.0, 0]
        self.tracking_point_pub.publish(msg)
        # rospy.loginfo("Sending empty message ...")


    ###### HELPERS ######
        
    def print_info(self, num_person, centers:list, areas:list):
        rospy.loginfo("Found %d people, center positions = %s, areas = %s" % (num_person, centers, areas))

    def get_max_bb_center(self, centers:list, areas:list) -> tuple:
        max_area = max(areas)
        max_id = areas.index(max_area)
        max_center = centers[max_id]
        return max_center

    def compute_centre(self, xmin, ymin, xmax, ymax):
        x = (xmin + xmax) / 2
        y = (ymin + ymax) / 2
        return x, y

    def compute_area(self, xmin, ymin, xmax, ymax):
        height = ymax - ymin
        width = xmax - xmin
        area = height * width
        return area



if __name__ == '__main__':

    finder = PersonFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        # cv2.destroyAllWindows()
        pass
    except rospy.ROSInterruptException:
        pass
