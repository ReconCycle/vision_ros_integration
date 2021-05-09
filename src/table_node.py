#! /usr/bin/env python

import rospy 
from robot_module_msgs.srv import CellVisualization
from rospy import Time

if __name__ == '__main__':
    rospy.init_node('table_node')
    rospy.wait_for_service('cell_layout_service')
    addTableSrv = rospy.ServiceProxy('cell_layout_service', CellVisualization)
    try:
        resp = addTableSrv(rospy.get_param('~childName'), rospy.get_param('~parentName'), 
                        rospy.get_param('~childSwitch'), rospy.get_param('~parentSwitch'),
                        'table.stl', rospy.get_param('~action'))
        rospy.loginfo(resp)
    except rospy.ServiceException as exc:
        rospy.loginfo("Service exception: " + str(exc))