#!/usr/bin/env python3

import rospy
from lab_f_traffic_bot.msg import Traffic
import numpy as np

class TrafficNode(object):
    def __init__(self):
        # Set up traffic status publisher
        self.traffic_status_pub = rospy.Publisher("/traffic_status", Traffic, queue_size=10)
        rospy.sleep(1)

    def run(self):
        # Once every 10 seconds
        rate = rospy.Rate(0.1)
        while (not rospy.is_shutdown()):
            # TODO: send traffic message
            traffic = Traffic()
            # Max Ranges: [-162 to 162, -103 to 90, -53 to 79, -100 to 117]

            #Joint 1 is horizontal (left to right)
            #Joint 2 is vertical (lower motor near base, up and down)
            #Joint 3 is vertical (top motor near arm, up and down)
            #Joint 4 is hand movement 
            traffic.joints = [0, 
                              np.random.randint(-103, 91),
                              np.random.randint(-53, 79),
                              np.random.randint(-100, 117)]
            # traffic.joints = [0,0,0,0]
            traffic.grippers = [0, 0]
            print(traffic.joints)

            self.traffic_status_pub.publish(traffic)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('traffic_controller')
    traffic_node = TrafficNode()
    traffic_node.run()
