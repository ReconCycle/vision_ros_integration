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
        resp = addTableSrv(2, 0, 'table.obj', 0)
        print(resp)
    except rospy.ServiceException as exc:
        print("Service exception: " + str(exc))
    rospy.spin()