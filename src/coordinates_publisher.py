#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

if __name__ == '__main__':
    rospy.init_node('coord_publisher', anonymous=True)
    pub = rospy.Publisher('/test_coordinates', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    msg = Float32MultiArray()
    msg.data = [0.1, 0.1, 0]

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()