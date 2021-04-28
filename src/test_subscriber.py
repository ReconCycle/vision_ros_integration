#!/usr/bin/env python
import rospy
from rospy import Time
from std_msgs.msg import String
import numpy as np


keywords = ['class_name', 'score', 'obb_corners', 'obb_center', 'obb_rot_quat']

def callbackReceivedData(msg):
    receivedString = msg.data
    numberOfClasses = receivedString.count(keywords[0])
    # Find indexes of 'class_name' substrings:
    idxClassName = findIdxSubstring(receivedString, keywords[0])
    # Find indexes of 'score' substrings (after each class_name comes score):
    idxScore = findIdxSubstring(receivedString, keywords[1])
    # Find indexes of 'obb_corners' substring:
    idxCorners = findIdxSubstring(receivedString, keywords[2])
    # Find indexes of 'obb_center' substrings:
    idxCenter = findIdxSubstring(receivedString, keywords[3])
    # Find indexes of 'obb_rot_quat' substrings:
    idxQuat = findIdxSubstring(receivedString, keywords[4])

    # Find actual class names (between substring 'class_name' and 'score'):
    classNames = []
    for idx in range(len(idxClassName)):
        className = receivedString[idxClassName[idx] + len(keywords[0]) + 4 : idxScore[idx] - 4]
        classNames.append(className)

    # Find actual corners coordinates (between substrings 'obb_corners' and 'obb_center'):
    cornerRawCoords = []
    cornerCoords = []
    for idx in range(len(idxCorners)):
        corners = receivedString[idxCorners[idx] + len(keywords[2]) + 3 : idxCenter[idx] - 3]
        # Split by comma and get rid of all [ and ]:
        corners = corners.replace('[', '')
        corners = corners.replace(']', '')
        corners = corners.split(',')
        # Convert to float:
        for corner in corners:
            corner = float(corner)
            cornerRawCoords.append(corner)

    # Create tuples (x, y) for each corner:
    for i in range(len(cornerRawCoords)/10):
        cornerCoords.append(((cornerRawCoords[10*i], cornerRawCoords[10*i+1]), 
                            (cornerRawCoords[10*i+2], cornerRawCoords[10*i+3]),
                            (cornerRawCoords[10*i+4], cornerRawCoords[10*i+5]),
                            (cornerRawCoords[10*i+6], cornerRawCoords[10*i+7]),
                            (cornerRawCoords[10*i+8], cornerRawCoords[10*i+9])))
    
   
    # Find actual center coordinates (between substrings 'obb_center' and 'obb_quat_rot'):
    centerRawCoords = []
    centerCoords = []
    for idx in range(len(idxCenter)):
        # Split by comma and get rid of all [ and ]
        centers = receivedString[idxCenter[idx] + len(keywords[3]) + 3 : idxQuat[idx] - 3]
        centers = centers.replace('[', '')
        centers = centers.replace(']', '')
        centers = centers.split(',')
        # Convert to float:
        for center in centers:
            center = float(center)
            centerRawCoords.append(center)
    # Create tuples (x, y) for each center:
    for i in range(len(centerRawCoords) / 2):
        centerCoords.append((centerRawCoords[2*i], centerRawCoords[2*i+1]))

    # Find actual quaternions (after substrings 'obb_rot_quat')
    quaternionsRaw = []
    quaternionsVals = []
    for idx in range(len(idxQuat)):
        if (idx < len(idxQuat) - 1):
            quaternions = receivedString[idxQuat[idx] + len(keywords[4]) + 3 : idxClassName[idx + 1] - 5]
        else:
            quaternions = receivedString[idxQuat[idx] + len(keywords[4]) + 3 : -4]
        quaternions = quaternions.replace('[', '')
        quaternions = quaternions.replace(']', '')
        quaternions = quaternions.split(',')
        for quat in quaternions:
            quat = float(quat)
            quaternionsRaw.append(quat)
    for i in range(len(quaternionsRaw) / 4):
        quaternionsVals.append((quaternionsRaw[4*i], quaternionsRaw[4*i+1], quaternionsRaw[4*i+2], quaternionsRaw[4*i+3]))

        

def findIdxSubstring(string, substring):
    return [i for i in range(len(string)) if string.startswith(substring, i)]



if __name__ == '__main__':
    rospy.init_node('test_vision_sub')
    rospy.loginfo('Node started')
    sub = rospy.Subscriber('/vision_pipeline/data', String, callbackReceivedData)

    rospy.spin()