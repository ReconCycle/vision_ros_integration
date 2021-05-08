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
parentTableIdx = []
activeTableDict = {}

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

# def createChildTableMarkerArray():
#     global activeTableDict
#     markerArray = MarkerArray()
#     for activeTable in activeTableDict:
#         marker = Marker()
#         marker.header.frame_id = 

# def createChildTableTf():
#     global activeTableDict
#     for table in activeTableDict:


def tableHandler(req):
    global activeTableDict
    success = False 
    # Add table if not already added. Index 0 is reserved for base table.
    if (req.action == 0 and req.childIdx not in childTableIdx and req.childIdx != 0):
        childTableIdx.append(req.childIdx)
        activeTableDict['table_' + str(req.childIdx)] = (req.childIdx, req.parentIdx)
        success = True
    
    # Remove table if previously added.
    if (req.action == 1 and req.childIdx in childTableIdx and req.childIdx != 0):
        childTableIdx.remove(req.childIdx)
        activeTableDict.pop('table_' + str(req.childIdx))
        success = True

    message = 'Tables: ' + str(activeTableDict)
    return success, message

if __name__ == '__main__':
    rospy.init_node('base_table_node')
    baseTablePublisher = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)
    baseTableMarker = createBaseTableMarker()
    # Add base table to active tables dictionary.
    activeTableDict['base_table'] = (0, 0)

    # Table manager service
    tableService = rospy.Service('robot_cell_visualization_service', robot_cell_visualization, tableHandler)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        baseTablePublisher.publish(baseTableMarker)
        rate.sleep()

