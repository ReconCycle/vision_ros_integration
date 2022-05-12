#!/usr/bin/env python

import rospy
import math
from rospy import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf import TransformBroadcaster
# import vp_data_parser as parser
import json


receivedString = ''
freq = 100
objectScaleZ = 0.03
textColor = (1.0, 0.0, 1.0, 1.0)
textScaleZ = 0.05
textOffsetZ = 0.15

# Read node configuration parameters
def readConfigParams():
    keywords = rospy.get_param("/VisionPipeline/keywords")
    colorsDict = rospy.get_param("/VisionPipeline/object_color_dict")
    # Read namespace
    cameraNs = rospy.get_namespace()
    rospy.loginfo("ROS namespace: {}".format(cameraNs))
    # Local parameter for each camera ns.
    cameraTf = rospy.get_param(cameraNs + "/vp_data_visualization/camera_tf")
    rospy.loginfo("Object TF name: {}".format(cameraTf))

    return keywords, colorsDict, cameraTf, cameraNs

# Receive message from /vision_pipeline/data topic
def callbackReceivedData(msg):
    global receivedString
    receivedString = msg.data
    #print('Received detection string:', receivedString)

# Keep track of active frames - if object is duplicated, add a number to its name.
def trackActiveFramesAndColors(activeClasses,cameraTf):
    activeFrames = []
    activeColors = []

    tmp_k = {} # Dictionary for keeping track of current index of each particular class (battery, hca_front, etc)

    for activeClass in activeClasses:
        try:
            activeColors.append(colorsDict[0][activeClass])
            if activeClass in tmp_k.keys():
                tmp_k[activeClass] += 1
                activeClass = activeClass + str(tmp_k[activeClass])
            else:
                tmp_k[activeClass] = 0

            activeFrames.append(activeClass+ '_' + cameraTf)
            
        except Exception as e:
            pass
    
    return activeFrames, activeColors

def sendTfTransform(activeFrames, centers, quaternions, cameraTf):

    transformBroadcaster = TransformBroadcaster()
    for i in range(len(activeFrames)):
        translation = (centers[i][0], centers[i][1], 0)
        transformBroadcaster.sendTransform(translation, quaternions[i], Time.now(), activeFrames[i], cameraTf)

def calculateObjectEdges(activeCorners):

    activeEdges = []
    for corners in activeCorners:
        xEdge = math.sqrt((corners[1][0] - corners[2][0])**2 + (corners[1][1] - corners[2][1])**2)
        yEdge = math.sqrt((corners[0][0] - corners[1][0])**2 + (corners[0][1] - corners[1][1])**2)
        
        longEdge = max(xEdge,yEdge)
        shortEdge = min(xEdge,yEdge)
        activeEdges.append((shortEdge, longEdge))
        
        # activeEdges.append((xEdge, yEdge))

    return activeEdges

def prepareObjectArray(activeFrames, activeColors, activeEdges, cameraNs):
    markerArray = MarkerArray()
    markerArray.markers = []
    for i in range(len(activeFrames)):
        marker = Marker()
        marker.header.frame_id = activeFrames[i]
        marker.header.stamp = Time.now()
        marker.ns = cameraNs + 'object'
        marker.id = i
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.z = objectScaleZ / 2
        marker.pose.orientation.w = 1.0
        marker.scale.x = activeEdges[i][0]
        marker.scale.y = activeEdges[i][1]
        marker.scale.z = objectScaleZ
        marker.color.r = activeColors[i][0]
        marker.color.g = activeColors[i][1]
        marker.color.b = activeColors[i][2]
        marker.color.a = activeColors[i][3]
        marker.lifetime = rospy.Duration(0.008)
        markerArray.markers.append(marker)
    
    return markerArray

def prepareTextArray(activeFrame, cameraNs):
    markerArray = MarkerArray()
    markerArray.markers = []
    for i in range(len(activeFrames)):
        marker = Marker()
        marker.header.frame_id = activeFrames[i]
        marker.header.stamp = Time.now()
        marker.ns = cameraNs + 'text'
        marker.id = i
        marker.type = marker.TEXT_VIEW_FACING
        marker.scale.z = textScaleZ
        marker.color.r = textColor[0]
        marker.color.g = textColor[1]
        marker.color.b = textColor[2]
        marker.color.a = textColor[3]
        marker.pose.position.z = textOffsetZ
        marker.pose.orientation.z = 1.0
        marker.text = activeFrames[i]
        marker.lifetime = rospy.Duration(0.008)
        markerArray.markers.append(marker)

    return markerArray
    
def filterMarkerArray(markerArray, outputClass = 'battery'):
    """ After we get an input marker array, we only select markers of certain class (input_class, for example 'battery') and return those,
    so we can for example show only battery markers"""
    
    assert outputClass in ['battery', 'hca_front', 'hca_back', 'hca_side']
    
    new_marker_array = MarkerArray()
    new_marker_array.markers = []
    
    for marker in markerArray.markers:
        if outputClass in marker.header.frame_id:
            new_marker_array.markers.append(marker)
    
    return new_marker_array

if __name__ == '__main__':
    rospy.init_node('vision_pipeline_data_visualization')

    keywords, colorsDict, cameraTf, cameraNs = readConfigParams()  
    sub = rospy.Subscriber('/vision_pipeline/data', String, callbackReceivedData)
    
    batteryArrayPub  = rospy.Publisher('/visualization_marker_array/battery', MarkerArray, queue_size = 20)
    hcaFrontArrayPub =  rospy.Publisher('/visualization_marker_array/hca_front', MarkerArray, queue_size = 20)
    hcaBackArrayPub =  rospy.Publisher('/visualization_marker_array/hca_back', MarkerArray, queue_size = 20)
    hcaSideArrayPub = rospy.Publisher('/visualization_marker_array/hca_side', MarkerArray, queue_size = 20)
    
    objectArrayPub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 20)
    
    textArrayPub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 20)

    rate = rospy.Rate(freq)

    while not rospy.is_shutdown():
        #### Parsing code
        # cleanedString = parser.cleanString(receivedString, ['[', ']', '{', '}', '"'])
        # idxClassName, idxScore, idxCorners, idxCenter, idxQuat, idxTrackingId, idxTrackingScore = parser.findIndexes(cleanedString, keywords)
        # activeClasses = parser.parseClassNames(cleanedString, idxClassName, idxScore, keywords[0])
        # corners = parser.parseCorners(cleanedString, idxCorners, idxCenter, keywords[2])
        # centers = parser.parseCenters(cleanedString, idxCenter, idxQuat, keywords[3])
        # quaternions = parser.parseQuaternions(cleanedString, idxQuat, idxTrackingId, keywords[4])
        #######################

        if receivedString == '':
            continue

        detections_str = json.loads(receivedString)
        activeClasses = []
        corners = []
        centers = []
        quaternions = []
        for detection in detections_str:
            if detection['label'] != 'hca_back':
                continue
            activeClasses.append(detection['label'])
            corners.append(detection['obb_corners_meters'])
            centers.append(detection['obb_center_meters'])
            quaternions.append(detection['obb_rot_quat'])

        activeEdges = calculateObjectEdges(corners)
        activeFrames, activeColors = trackActiveFramesAndColors(activeClasses, cameraTf)
        sendTfTransform(activeFrames, centers, quaternions, cameraTf)
        
        objectArray = prepareObjectArray(activeFrames, activeColors, activeEdges, cameraNs)
        textArray = prepareTextArray(activeFrames, cameraNs)
        
        batteryArray = filterMarkerArray(objectArray, outputClass = 'battery')
        hcaFrontArray = filterMarkerArray(objectArray, outputClass = 'hca_front')
        hcaBackArray =  filterMarkerArray(objectArray, outputClass = 'hca_back')
        hcaSideArray = filterMarkerArray(objectArray, outputClass = 'hca_side')

        objectArrayPub.publish(objectArray)
        textArrayPub.publish(textArray)
        
        batteryArrayPub.publish(batteryArray)
        hcaFrontArrayPub.publish(hcaFrontArray)
        hcaBackArrayPub.publish(hcaBackArray)
        hcaSideArrayPub.publish(hcaSideArray)

        rate.sleep()
