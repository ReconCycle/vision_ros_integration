#!/usr/bin/env python

import rospy
import tf
from tf import TransformBroadcaster
from rospy import Time
from std_msgs.msg import Float32MultiArray
import numpy as np


receivedCoords = (0.0, 0.0, 0.0)
receivedEulers = (0.0, 0.0, 0.0)
cuboidZOffset = 3 
scale = 0.01
def callbackReceiveCoords(msg):
    global receivedCoords
    global receivedEulers
    receivedCoords = msg.data[0:3]
    receivedEulers = msg.data[3:]


def moveCuboid():
    b = TransformBroadcaster()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        translation = (receivedCoords[0], receivedCoords[1], receivedCoords[2] + cuboidZOffset * scale)       
        rotation = receivedEulers
        rotation = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        print(rotation)
        b.sendTransform(translation, rotation, Time.now(), 'cuboid_link', 'panda_1/world')
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tfTest', anonymous=True)
    sub = rospy.Subscriber('/test_coordinates', Float32MultiArray, callbackReceiveCoords)
    moveCuboid()

    rospy.spin()





    
