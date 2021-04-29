#!/usr/bin/env python
import rospy
from rospy import Time
from std_msgs.msg import String
import numpy as np
import vp_data_parser as parser


keywords = ['class_name', 'score', 'obb_corners', 'obb_center', 'obb_rot_quat']
receivedString = ''
numberOfClasses = 0

def callbackReceivedData(msg):
    global receivedString
    global numberOfClasses
    receivedString = msg.data
    numberOfClasses = receivedString.count(keywords[0])


if __name__ == '__main__':
    rospy.init_node('test_vision_sub')
    rospy.loginfo('Node started')
    sub = rospy.Subscriber('/vision_pipeline/data', String, callbackReceivedData)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        cleanedString = parser.cleanString(receivedString, ['[', ']', '{', '}', '"'])
        idxClassName, idxScore, idxCorners, idxCenter, idxQuat = parser.findIndexes(cleanedString, keywords)
        corners = parser.parseCorners(cleanedString, idxCorners, idxCenter, keywords[2])
        centers = parser.parseCenters(cleanedString, idxCenter, idxQuat, keywords[3])
        quaternions = parser.parseQuaternions(cleanedString, idxQuat, idxClassName, keywords[4])
        rate.sleep()
