#! /usr/bin/env python

import rospy 
from robot_module_msgs.srv import CellVisualization
from visualization_msgs.msg import Marker
from rospy import Time
import tf

tablesData = []
tableWidth = 0.6
parentSwitchDict = {
    'none' : [0, 0],
    'north' : [0, 1],
    'east' : [1, 0],
    'south' : [0, -1],
    'west' : [-1, 0]
}

def layoutHandler(req):
    global tablesData  
    success = False
    message = ''
    if req.action == 'add':
        message = 'Table has been already added.'
        if (req.childName, req.parentName, req.parentSwitch) not in tablesData:
            tablesData.append((req.childName, req.parentName, req.parentSwitch))
            success = True
            message = 'Attach table ' + req.childName + ' to table ' + req.parentName + ' on parent switch ' + req.parentSwitch
    if req.action == 'remove':
        message = 'Table has never been added or has been already removed.'
        if (req.childName, req.parentName, req.parentSwitch) in tablesData:
            tablesData.remove((req.childName, req.parentName, req.parentSwitch))
            success = True
            message = 'Detach table ' + req.childName + ' from table ' + req.parentName + ' on parent switch ' + req.parentSwitch

    return success, message

# Base table is visualized in world frame, all other tables are visualized in base_table frame.
def visualizeTable(broadcaster, listener, markerPublisher):
    marker = Marker()
    for table in tablesData:
        # Put base table in world origin.
        if table[0] == 'base_table' and table[1] == 'world':
            translation = (0.0, 0.0, 0.0)
            quaternions = (0.0, 0.0, 0.0, 1.0)
            baseFrame = table[1]
        # Put every other tables in base_table frame.
        else:
            baseFrame = 'base_table'
            # Get parent table transform from base_table.
            parentTfTranslation, parentTfRotation = listener.lookupTransform(baseFrame, table[1], Time(0))
            # Calculate child table transform, based on parent table transform.
            translation = (parentTfTranslation[0] + parentSwitchDict[table[2]][0] * tableWidth, 
                            parentTfTranslation[1] + parentSwitchDict[table[2]][1] * tableWidth, 
                            parentTfTranslation[2])
            quaternions = parentTfRotation
                   
        # Send transform
        broadcaster.sendTransform(translation, quaternions, Time.now(), table[0], baseFrame)
        # Send marker
        marker.header.frame_id = table[0]
        marker.header.stamp = Time.now()
        marker.ns = table[0]
        marker.id = 1
        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = 'package://reconcycle_description/meshes/table.stl'
        marker.mesh_use_embedded_materials = True
        marker.action = marker.ADD
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.pose.orientation.x = 1.0
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.2)
        markerPublisher.publish(marker)



if __name__ == '__main__':
    rospy.init_node('cell_layout_manager_node')
    cellLayoutService = rospy.Service('cell_layout_service', CellVisualization, layoutHandler)
    transformBroadcaster = tf.TransformBroadcaster()
    transformListener = tf.TransformListener()
    markerPub = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        visualizeTable(transformBroadcaster, transformListener, markerPub)
        
        rate.sleep()
