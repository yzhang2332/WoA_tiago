#!/usr/bin/env python

import rospy
from std_msgs.msg import String

from nav_function import NavigationClient


def string_publisher():

    initial_poi = str(input("Enter the current poi name: "))
    nav = NavigationClient(initial_poi)
    
    running = True

    while running:
        poi_name = str(input("Enter the poi name: "))
        if poi_name == "q":
            break
        rospy.loginfo(poi_name)
        nav.run(poi_name)
    


if __name__ == '__main__':
    try:
        string_publisher()
    except rospy.ROSInterruptException:
        pass
