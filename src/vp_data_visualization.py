#!/usr/bin/env python
import rospy
from rospy import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf import TransformBroadcaster
import vp_data_parser as parser



# Parser variables
keywords = ['class_name', 'score', 'obb_corners', 'obb_center', 'obb_rot_quat']
receivedString = ''
numberOfClasses = 0

freq = 100


# Receive message from /vision_pipeline/data topic
def callbackReceivedData(msg):
    global receivedString
    global numberOfClasses
    receivedString = msg.data
    numberOfClasses = receivedString.count(keywords[0])

# Keep track of active frames - if object is duplicated, add a number to its name.
def trackActiveFrames(activeClasses):
    k = 0
    activeFrames = []
    for activeClass in activeClasses:
        if activeClass in activeFrames:
            k += 1
            activeClass = activeClass + str(k)
        k = 0
        activeFrames.append(activeClass)
    
    return activeFrames

def sendTFTransform(activeFrames, centers, quaternions):
    transformBroadcaster = TransformBroadcaster()
    for i in range(len(activeFrames)):
        translation = (centers[i][0], centers[i][1], 0)
        transformBroadcaster.sendTransform(translation, quaternions[i], Time.now(), activeFrames[i], 'panda_1/world')

def prepareObjectArray(activeFrames):
    markerArray = MarkerArray()
    markerArray.markers = []
    for i in range(len(activeFrames)):
        marker = Marker()
        marker.header.frame_id = activeFrames[i]
        marker.header.stamp = Time.now()
        marker.ns = activeFrames[i] + 'object'
        marker.id = i
        marker.type = marker.MESH_RESOURCE
        marker.action = marker.ADD
        marker.mesh_resource = "package://reconcycle_description/meshes/cuboid.stl"
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.01)
        markerArray.markers.append(marker)
    
    return markerArray

def prepareTextArray(activeFrame):
    markerArray = MarkerArray()
    markerArray.markers = []
    for i in range(len(activeFrames)):
        marker = Marker()
        marker.header.frame_id = activeFrames[i]
        marker.header.stamp = Time.now()
        marker.ns = activeFrames[i] + 'text'
        marker.id = i
        marker.type = marker.TEXT_VIEW_FACING
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.z = 0.15
        marker.pose.orientation.z = 1.0
        marker.text = activeFrames[i]
        marker.lifetime = rospy.Duration(0.01)
        markerArray.markers.append(marker)

    return markerArray

    

if __name__ == '__main__':
    rospy.init_node('vision_pipeline_data_visualization')
    rospy.loginfo('Node started')
    sub = rospy.Subscriber('/vision_pipeline/data', String, callbackReceivedData)
    objectArrayPub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 20)
    textArrayPub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 20)
    rate = rospy.Rate(freq)
    # Keep track of all created frames.

    while not rospy.is_shutdown():
        cleanedString = parser.cleanString(receivedString, ['[', ']', '{', '}', '"'])
        idxClassName, idxScore, idxCorners, idxCenter, idxQuat = parser.findIndexes(cleanedString, keywords)
        activeClasses = parser.parseClassNames(cleanedString, idxClassName, idxScore, keywords[0])
        corners = parser.parseCorners(cleanedString, idxCorners, idxCenter, keywords[2])
        centers = parser.parseCenters(cleanedString, idxCenter, idxQuat, keywords[3])
        quaternions = parser.parseQuaternions(cleanedString, idxQuat, idxClassName, keywords[4])
        
        activeFrames = trackActiveFrames(activeClasses)
        sendTFTransform(activeFrames, centers, quaternions)
        objectArray = prepareObjectArray(activeFrames)
        textArray = prepareTextArray(activeFrames)

        objectArrayPub.publish(objectArray)
        textArrayPub.publish(textArray)


        print(activeFrames)
        rate.sleep()