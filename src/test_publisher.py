#!/usr/bin/env python
import rospy
from rospy import Time
from robot_module_msgs.msg import VisionData
from std_msgs.msg import Float32MultiArray


if __name__ == '__main__':
    rospy.init_node('test_publisher')
    visionData = VisionData()
    pub = rospy.Publisher('/vision_test', VisionData, queue_size = 10)
    rate = rospy.Rate(3)
    visionData.numberOfObjects = 4
    visionData.objectIndex = [0, 1, 2, 3]
    visionData.coordinates = [0.0, -0.8, 0.0, 0.0, -0.2, 0.0, 0.0, -0.4, 0.0, 0.0, -0.6, 0.0]
    visionData.eulerAngles = [0.0, 0.0, 0.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.75, 0.0, 0.0, 0.0]

    while not rospy.is_shutdown():
        
        pub.publish(visionData)
        rate.sleep()

