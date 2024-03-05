#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class HalfTurn:

    def __init__(self):

        
        self.pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

        self.turn_duration = 5.0                         # seconds
        self.buffer = 0.2           # seconds
        self.turn_rate = math.pi / self.turn_duration   # rad/s [clockwise]

        # self.start_timestamp = time()
        self.start_timestamp = rospy.get_time()   # since epoch
    

    def run(self):
        
        now = rospy.get_time()
        diff = now-self.start_timestamp
        while diff < self.turn_duration + self.buffer:
            twist = Twist()
            twist.angular.z = self.turn_rate
            print("publishing %.3f" % self.turn_rate)
            self.pub_cmd.publish(twist)
            now = rospy.get_time()
            diff = now-self.start_timestamp
            print(diff)


 
def main():
    rospy.init_node('half_turn', anonymous=True)
    turner = HalfTurn()
    turner.run()

if __name__ == '__main__':
    main()
    
