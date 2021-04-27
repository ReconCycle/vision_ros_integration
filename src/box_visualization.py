#!/usr/bin/env python

import rospy
import tf
from tf import TransformBroadcaster
from rospy import Time
from robot_module_msgs.msg import VisionData
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


cuboidZOffset = 3 
scale = 0.01
objectsDict = {
    0 : 'remote_control',
    1 : 'battery',
    2 : 'pcb',  
    3 : 'internals'
}
numberOfObjects = 0
objectIdx = []
coords = []
eulers = []

# Read message from /vision_test:
def callbackReceiveCoords(msg):
    global numberOfObjects
    global objectIdx
    global coords
    global eulers
    numberOfObjects = msg.numberOfObjects
    objectIdx = msg.objectIndex
    coords = msg.coordinates
    eulers = msg.eulerAngles

# Prepare TFMessage for /tf topic for each object:
def prepareTFMessage(idx):
    frame_name = objectsDict[objectIdx[idx]]
    translation = coords[idx*3:idx*3+3]
    rotation = eulers[idx*3:idx*3+3]
    translation = (translation[0], translation[1], translation[2] + cuboidZOffset * scale)
    rotation = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
    
    return frame_name, translation, rotation

# Prepare Marker msg for each object:
def prepareMarkerArrayMessage(frame_name, idx):
    marker = Marker()
    marker.header.frame_id = frame_name
    marker.header.stamp = Time.now()
    marker.ns = frame_name + str(idx)
    marker.type = marker.TEXT_VIEW_FACING
    marker.scale.z = 0.05
    marker.color.r = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.pose.position.z = 0.05
    marker.pose.orientation.z = 1.0
    marker.text = frame_name
    marker.id = idx

    return marker

#  Send TFMessage and MarkerArray message on topics:
def sendTfMarker():
    transformBroadcaster = TransformBroadcaster()
    markerPub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 10)
    markerArray = MarkerArray()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        markerArray.markers = []
        for i in range(numberOfObjects):
            frame_name, translation, rotation = prepareTFMessage(i)
            transformBroadcaster.sendTransform(translation, rotation, Time.now(), frame_name, 'panda_1/world')

            marker = prepareMarkerArrayMessage(frame_name, i)
            markerArray.markers.append(marker)
            markerPub.publish(markerArray)

        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tfTest', anonymous=True)
    sub = rospy.Subscriber('/vision_test', VisionData, callbackReceiveCoords)
    sendTfMarker()

    rospy.spin()





    
