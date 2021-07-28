#! /usr/bin/env python

import rospy
from robot_module_msgs.srv import StlFileManager


def stlHandler(req):
    global thisTableName
    global stlFilename

    success = False
    stl = ''
    if req.tableName == thisTableName:
        stl = stlFilename
        success = True
    
    return stl, success

if __name__ == '__main__':
    rospy.init_node('rpi_table_node', anonymous = True)
    thisTableName = rospy.get_param('~tableName')
    stlFilename = rospy.get_param('~stlFile')
    stlFileService = rospy.Service('stl_file_service_' + thisTableName, StlFileManager, stlHandler)

    rospy.spin()