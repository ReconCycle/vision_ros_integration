#! /usr/bin/env python

import rospy 
from robot_module_msgs.srv import robot_cell_visualization
from tf import TransformBroadcaster
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rospy import Time

baseTableFile = 'table.stl'
baseTableFrame = 'base_table'

# Array of all child tables indexes.
childTableIdx = []

def createBaseTableMarker():
    marker = Marker()
    marker.header.frame_id = baseTableFrame
    marker.header.stamp = Time.now()
    marker.id = 1
    marker.ns = baseTableFrame + '_ns'
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = 'package://reconcycle_description/meshes/' + baseTableFile
    marker.mesh_use_embedded_materials = True
    marker.action = marker.ADD
    marker.pose.orientation.x = 1.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.001
    marker.scale.y = 0.001
    marker.scale.z = 0.001

    return marker

def tableHandler(req):
    success = False 
    if (req.action == 0 and req.childIdx not in childTableIdx):
        childTableIdx.append(req.childIdx)
        success = True
    
    if (req.action == 1 and req.childIdx in childTableIdx):
        childTableIdx.remove(req.childIdx)
        success = True

    message = 'Number of child tables: ' + str(len(childTableIdx))
    return success, message

if __name__ == '__main__':
    rospy.init_node('base_table_node')
    baseTablePublisher = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)
    baseTableMarker = createBaseTableMarker()

    # Table manager service
    tableService = rospy.Service('robot_cell_visualization_service', robot_cell_visualization, tableHandler)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        baseTablePublisher.publish(baseTableMarker)
        rate.sleep()

