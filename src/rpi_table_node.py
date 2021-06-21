#! /usr/bin/env python

import rospy
from robot_module_msgs.srv import StlFileManager


def stlHandler(req):
    global thisTableName
    success = False
    stlFilename = ''
    if req.tableName == thisTableName:
        stlFilename = thisTableName
        success = True
    
    return stlFilename, success




if __name__ == '__main__':
    rospy.init_node('rpi_table_node', anonymous = True)
    thisTableName = rospy.get_param('~tableName')
    stlFileService = rospy.Service('stl_file_service_' + thisTableName, StlFileManager, stlHandler)

    rospy.spin()