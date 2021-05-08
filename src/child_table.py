#! /usr/bin/env python

import rospy 
from robot_module_msgs.srv import robot_cell_visualization
from rospy import Time

if __name__ == '__main__':
    rospy.init_node('child_table_node')
    print('Node started')
    rospy.wait_for_service('robot_cell_visualization_service')
    addTableSrv = rospy.ServiceProxy('robot_cell_visualization_service', robot_cell_visualization)
    try:
        # Index 0 is reserved for base table.
        resp = addTableSrv(rospy.get_param('~childIdx'), rospy.get_param('~parentIdx'), 'table.stl', rospy.get_param('~action'))
        print(resp)
    except rospy.ServiceException as exc:
        print("Service exception: " + str(exc))
    rospy.spin()