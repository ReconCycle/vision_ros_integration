#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float32MultiArray
import tf2_ros
import geometry_msgs.msg




receivedCoords = [0.0, 0.0, 0.0]
def callbackReceivedMsg(msg):
    global receivedCoords
    receivedCoords = msg.data
    br = tf2_ros.TransformBroadcaster()
    tf2Stamp = geometry_msgs.msg.TransformStamped()
    tf2Stamp.header.stamp = rospy.Time.now()
    tf2Stamp.header.frame_id = "world"
    tf2Stamp.child_frame_id = "table_rob_1"
    tf2Stamp.transform.translation.x = receivedCoords[0]
    tf2Stamp.transform.translation.y = receivedCoords[1]
    tf2Stamp.transform.translation.z = 0.0
    rotationEuler = tf.transformations.quaternion_from_euler(0, 0, 0)
    rospy.loginfo(rotationEuler)
    tf2Stamp.transform.rotation.x = rotationEuler[0]
    tf2Stamp.transform.rotation.y = rotationEuler[1]
    tf2Stamp.transform.rotation.z = rotationEuler[2]
    tf2Stamp.transform.rotation.w = rotationEuler[3]
    br.sendTransform(tf2Stamp)


if __name__ == '__main__':
    rospy.init_node('coord_subscriber', anonymous=True)
    rospy.Subscriber('/test_coordinates', Float32MultiArray, callbackReceivedMsg)

    rospy.spin()





    
