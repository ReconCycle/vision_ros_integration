#! /usr/bin/env python
import rospy
from robot_module_msgs.srv import StlFileManager

tableName = ''

def stlHandler(req):
    global tableName
    success = False
    stlFilename = ''
    if req.tableName == tableName:
        stlFilename = tableName
        success = True
    
    return stlFilename, success




if __name__ == '__main__':
    rospy.init_node('rpi_table_node', anonymous = True)
    tableName = rospy.get_param('~tableName')
    stlFileService = rospy.Service('stl_file_service', StlFileManager, stlHandler)

    rospy.spin()