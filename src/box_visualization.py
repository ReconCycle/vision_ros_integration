#!/usr/bin/env python

import rospy
import tf
from tf import TransformBroadcaster
from rospy import Time
from std_msgs.msg import Float32MultiArray
import numpy as np
from visualization_msgs.msg import Marker


receivedCoords = (0.0, 0.0, 0.0)
receivedEulers = (0.0, 0.0, 0.0)
cuboidZOffset = 3 
scale = 0.01
def callbackReceiveCoords(msg):
    global receivedCoords
    global receivedEulers
    receivedCoords = msg.data[0:3]
    receivedEulers = msg.data[3:]


def moveCuboidWithMarker():
    b = TransformBroadcaster()
    markerPub = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)
    marker = Marker()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # Prepare TFMessage
        translation = (receivedCoords[0], receivedCoords[1], receivedCoords[2] + cuboidZOffset * scale)       
        rotation = receivedEulers
        rotation = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        b.sendTransform(translation, rotation, Time.now(), 'cuboid_link', 'panda_1/world')
        
        # Prepare Marker message
        marker.header.stamp = Time.now()
        marker.header.frame_id = "cuboid_link"
        marker.ns = ""
        marker.id = 0
        marker.type = 9
        marker.pose.position.z = 0.05
        marker.pose.orientation.z = 1.0
        marker.scale.z = 0.05
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = "CUBOID"
        markerPub.publish(marker)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tfTest', anonymous=True)
    sub = rospy.Subscriber('/test_coordinates', Float32MultiArray, callbackReceiveCoords)
    moveCuboidWithMarker()

    rospy.spin()





    
