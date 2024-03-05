#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool


def follow_flag_publisher():

    # Initialize the ROS Node named 'bool_publisher_node'
    rospy.init_node('follow_flag_publisher', anonymous=True)

    # Create a publisher object that will publish messages of type Bool
    # on the topic 'bool_topic'
    pub = rospy.Publisher('/follow_person_flag', Bool, queue_size=1)

    # Set the loop rate to 100 Hz
    rate = rospy.Rate(100)


    while not rospy.is_shutdown():

        # Define the boolean message to be True or False
        bool_msg = Bool()
        bool_msg.data = True

        # Log the message being published (for debugging purposes)
        rospy.loginfo(f"Publishing: {bool_msg.data}")

        # Publish the message
        pub.publish(bool_msg)

        # Sleep for the remainder of the loop rate
        rate.sleep()


if __name__ == '__main__':
    try:
        follow_flag_publisher()
    except rospy.ROSInterruptException:
        pass
