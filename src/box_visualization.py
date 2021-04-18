#!/usr/bin/env python

import rospy
from tf import TransformBroadcaster
from std_msgs.msg import Float32MultiArray
import tf2_ros
import geometry_msgs.msg
from rospy import Time


def moveCuboid():
    b = TransformBroadcaster()
    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0, 1.0)
    rate = rospy.Rate(5)
    x, y = 0.0, 0.0
    while not rospy.is_shutdown():
        if x >= 2:
            x, y = 0.0, 0.0
        x += 0.1
        y += 0.1
        translation = (x, y, 0.0)
        b.sendTransform(translation, rotation, Time.now(), 'cuboid', 'panda_1/world')
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tfBroadcaster', anonymous=True)
    moveCuboid()

    rospy.spin()





    
