#!/usr/bin/env python3
import time
import rospy
import math
from rospy import Time
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf import TransformBroadcaster
# import vp_data_parser as parser
from context_action_framework.msg import Detections, Detection, Gaps, Gap
from context_action_framework.types import Label
import json
import yaml
import copy
import tf

receivedString = ''
freq = 100
objectScaleZ = 0.03
textColor = (1.0, 0.0, 1.0, 1.0)
textScaleZ = 0.05
textOffsetZ = 0.15
receivedDetections = []
receivedGaps = []


class DetectionCheckClass:
    def __init__(self, x_percentage = 0.8, y_percentage = 0.8, cam_w = 640, cam_h = 480):
        """ Remove detections at the edge of the camera's vision zone since they are most often false or we are not interested in them.
        Args:
        detections : detections message from vision
        x_percentage : what percentage of X-pixels to keep around center. A value of 0.8 means 10% pixels on left edge and 10% on the right edge, altogether 20, will be removed
        (actually the detections there will be ignored)
        
        y -percentage - same
        
        cam_w - n pixels of camera img in x axis.
        cam_h - n pixels of camera img in y axis"""
        nx = (1 - x_percentage) / 2
        ny =  (1 - y_percentage) / 2

        self.x_min = int(nx * cam_w)
        self.x_max = int((1-nx) * cam_w)

        self.y_min = int(ny * cam_h)
        self.y_max = int((1-ny) * cam_h)
        
    def checkDetection(self, detection):
        """ Check a single detection and 
        returns :
        isvalid, detection
        isvalid == 1 if detection is valid obviously."""
        if (detection.center_px[0] > self.x_min) and (detection.center_px[0] < self.x_max) and (detection.center_px[1] > self.y_min) and (detection.center_px[1] < self.y_max):
            #rospy.loginfo("Accepting det., x:{}, y:{}".format(detection.center_px[0], detection.center_px[1]))
            return 1, detection
        else:
            #rospy.loginfo("Ignoring det., x:{}, y:{}".format(detection.center_px[0], detection.center_px[1]))
            return 0, detection

# Read node configuration parameters
def readConfigParams():
    
    keywords = rospy.get_param("/VisionPipeline/keywords")
    colorsDict = rospy.get_param("/VisionPipeline/object_color_dict")
    # Read namespace
    cameraNs = rospy.get_namespace()
    rospy.loginfo("ROS namespace: {}".format(cameraNs))
    # Local parameter for each camera ns.
    cameraTf = rospy.get_param(cameraNs + "/vp_data_visualization/camera_tf")
    rospy.loginfo("Object TF name: {}".format(cameraTf))
    
    vision_lookup_topic = rospy.get_param(cameraNs + "/vp_data_visualization/vision_lookup_topic")

    return keywords, colorsDict, cameraTf, cameraNs, vision_lookup_topic

def callbackReceivedGaps(msg):
    # Receive messages from /vision/realsense/gaps
    global receivedGaps
    receivedGaps = msg.gaps
    #print('Received detection string:', receivedString)
    

def callbackReceivedData(msg):
    # Receive message from /vision/basler/detections topic
    global detectionTime
    global receivedDetections
    receivedDetections = msg.detections
    detectionTime = time.time()
    #print('Received detection string:', receivedString)
        
        

def trackActiveFramesAndColors(activeClasses,activeDetections, cameraTf):
    # Keep track of active frames - if object is duplicated, add a number to its name.

    activeFrames = []
    activeColors = []

    #tmp_k = {} # Dictionary for keeping track of current index of each particular class (battery, hca_front, etc)
    #for activeClass in activeClasses:
    
    for i in range(0, len(activeClasses)):
        try:
            #id = activeDetections[i].id
            #if activeClass in tmp_k.keys():
            #    tmp_k[activeClass] += 1
            #    activeClass = activeClass + str(tmp_k[activeClass])
            #else:
            #    tmp_k[activeClass] = 0
            
            activeColors.append(colorsDict[0][activeClasses[i]])
            activeFrames.append(activeClasses[i]+'_'+str(activeDetections[i].id)+ '_' + cameraTf)
            
        except Exception as e:
            rospy.loginfo("Vision ros integration: exception {}".format(e))
            pass
    
    return activeFrames, activeColors

def sendTfTransform(transformBroadcaster, activeFrames, centers, quaternions, cameraTf):

    for i in range(len(activeFrames)):
        #translation = (centers[i].x, centers[i][1], 0)
        #rotation = (quaternions[i].x, quaternions[i].y, quaternions[i].z, quaternions[i].w)
        #transformBroadcaster.sendTransform(translation, quaternions[i], Time.now(), activeFrames[i], cameraTf)
        translation = (centers[i].x, centers[i].y, centers[i].z)   
        rotation=quaternions[i]

        transformBroadcaster.sendTransform(translation, rotation, Time.now(), activeFrames[i], cameraTf)
        
        
def calculateObjectEdges(activeCorners):

    activeEdges = []
    for corners in activeCorners:
        xEdge = math.sqrt((corners[1][0] - corners[2][0])**2 + (corners[1][1] - corners[2][1])**2)
        yEdge = math.sqrt((corners[0][0] - corners[1][0])**2 + (corners[0][1] - corners[1][1])**2)
        
        longEdge = max(xEdge,yEdge)
        shortEdge = min(xEdge,yEdge)
        
        #activeEdges.append((shortEdge, longEdge))
        activeEdges.append((longEdge,shortEdge))
        # activeEdges.append((xEdge, yEdge))

    return activeEdges

def prepareObjectArray(activeFrames, activeColors, activeEdges, cameraNs):
    markerArray = MarkerArray()
    markerArray.markers = []
    for i in range(len(activeFrames)):
        marker = Marker()
        marker.header.frame_id = activeFrames[i]
        marker.header.stamp = Time.now()
        marker.ns = cameraNs + 'object'
        marker.id = i
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.z = objectScaleZ / 2
        marker.pose.orientation.w = 1.0
        marker.scale.x = activeEdges[i][0]
        marker.scale.y = activeEdges[i][1]
        marker.scale.z = objectScaleZ
        marker.color.r = activeColors[i][0]
        marker.color.g = activeColors[i][1]
        marker.color.b = activeColors[i][2]
        marker.color.a = activeColors[i][3]
        marker.lifetime = rospy.Duration(0.008)
        markerArray.markers.append(marker)
    
    return markerArray

def prepareTextArray(activeFrame, cameraNs):
    markerArray = MarkerArray()
    markerArray.markers = []
    for i in range(len(activeFrames)):
        marker = Marker()
        marker.header.frame_id = activeFrames[i]
        marker.header.stamp = Time.now()
        marker.ns = cameraNs + 'text'
        marker.id = i
        marker.type = marker.TEXT_VIEW_FACING
        marker.scale.z = textScaleZ
        marker.color.r = textColor[0]
        marker.color.g = textColor[1]
        marker.color.b = textColor[2]
        marker.color.a = textColor[3]
        marker.pose.position.z = textOffsetZ
        marker.pose.orientation.z = 1.0
        marker.text = activeFrames[i]
        marker.lifetime = rospy.Duration(0.008)
        markerArray.markers.append(marker)

    return markerArray
    
def filterMarkerArray(markerArray, outputClass = 'battery'):
    """ After we get an input marker array, we only select markers of certain class (input_class, for example 'battery') and return those,
    so we can for example show only battery markers"""

    assert outputClass in ['battery', 'hca_front', 'hca_back', 'hca_side',
                           'pcb', 'internals', 'pcb_covered', 'plastic_clip']
    
    new_marker_array = MarkerArray()
    new_marker_array.markers = []
    
    for marker in markerArray.markers:
        if outputClass in marker.header.frame_id:
            new_marker_array.markers.append(marker)
    
    return new_marker_array

def uv_to_XY(u,v, P, Z = 0.35, get_z_from_rosparam = True):
    """Convert pixel coordinated (u,v) from realsense camera into real world coordinates X,Y,Z """
    
    assert P.shape == (3,4)
    
    if get_z_from_rosparam:
        Z = rospy.get_param('/vision/realsense_height')
    
    fx = P[0,0]
    fy = P[1,1]

    x = (u - (P[0,2])) / fx
    y = (v - (P[1,2])) / fy

    X = (Z * x)
    Y = (Z * y)
    Z = Z
    return X,Y, Z

def parse_calib_yaml(fn):
    """Parse camera calibration file (which is hand-made using ros camera_calibration) """
    # Testing converting camera coords to real world.
    with open(fn, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    data = data['Realsense']
    init_p = data['init_robot_pos']
    #print(data)
    w  = data['coeffs'][0]['width']
    h = data['coeffs'][0]['height']
    
    D = np.array(data['coeffs'][0]['D'])
    K = np.array(data['coeffs'][0]['K']).reshape(3,3)
    P = np.array(data['coeffs'][0]['P']).reshape(3,4)
    
    return init_p, D, K, P, w, h 

def obb_px_to_quat(px):
    
    p1 = np.array([px[0],px[1]])
    p2 = np.array([px[2],px[3]])
    
    p3 = np.array([px[4],px[5]])
    p4 = np.array([px[6],px[7]])
    
    d_edge_1 = np.linalg.norm(p2 - p1)
    
    d_edge_2 = np.linalg.norm(p4 - p1)
    
    if d_edge_1 > d_edge_2:
        long_edge = p1,p2
        short_edge = p1, p4
    else:
        long_edge = p1,p4
        short_edge = p1,p2
    
    # Assert that long_edge[0,0] is BIGGER than long_edge[1,0] (That one X is always bigger
    if long_edge[0][0] < long_edge[1][0]:
        nl = long_edge[1], long_edge[0]
        long_edge = nl
    
    # We get the angle of the LONG edge.
    a = long_edge[0][0] - long_edge[1][0]
    b = long_edge[0][1] - long_edge[1][1]
    
    
    angle = np.arctan2(b,a)
    #ang = 180*angle / np.pi
    #rospy.loginfo("angle:{}".format(ang))
    
    rot = tf.transformations.quaternion_from_euler(np.pi, 0, angle)
    #rot = rot[[3,0,1,2]]
    
    
    #rospy.loginfo("D1:{}".format(d_edge_1))
    #rospy.loginfo("D2:{}".format(d_edge_2))
    return rot


if __name__ == '__main__':
    
    rospy.init_node('vision_pipeline_data_visualization', anonymous = True)
    
    
    
    min_publish_dt = 0.1 # seconds
    
    max_detection_age_seconds = 0.5
    
    last_publish_time = time.time()
    

    keywords, colorsDict, cameraTf, cameraNs, vision_lookup_topic = readConfigParams() 
    #rospy.loginfo("Listening to:") 
    assert vision_lookup_topic in ['realsense', 'basler']
    #rospy.loginfo("cameraTf")
    #rospy.loginfo(cameraTf)
    
    #rospy.loginfo("cameraNs")
    #rospy.loginfo(cameraNs)
    
    topic = "/vision/%s/detections"%(vision_lookup_topic)
    #rospy.loginfo(topic)

    if vision_lookup_topic == 'basler':
        name_addition = "vision_table_zero"
        rospy.loginfo("WARNING: hardcoded -- y = 0.6 - detection.y -- in vp_data_visualization for BASLER ! FIX EET ! ")
        
    else:
        name_addition = 'realsense'
        fn = '/ros_ws/src/vision_ros_integration/config/realsense_calib/realsense_calib.yaml'
        init_p, D, K, P, w, h = parse_calib_yaml(fn)
        
        # Remove detections on the edge of img.
        detectionChecker = DetectionCheckClass(x_percentage = 0.8, y_percentage = 0.8, cam_w = 640, cam_h = 480)
        
        # For realsense we also want to read from the /vision/realsense/gaps topic
        gaps_topic = '/vision/%s/gaps'%(vision_lookup_topic)
        gaps_sub = rospy.Subscriber(gaps_topic, Gaps, callbackReceivedGaps)
        center_of_gap = Vector3() # We pre-init message so we dont init it every callback

    #name_addition = '_' + vision_lookup_topic # When we publish tf, we will name it for example battery_realsense. to this name_addition = '_realsense'
    
        
    #rospy.loginfo("listening to: {}".format(topic))
    sub = rospy.Subscriber(topic, Detections, callbackReceivedData)
    
    batteryArrayPub  = rospy.Publisher('/visualization_marker_array/battery', MarkerArray, queue_size = 20)
    hcaFrontArrayPub =  rospy.Publisher('/visualization_marker_array/hca_front', MarkerArray, queue_size = 20)
    hcaBackArrayPub =  rospy.Publisher('/visualization_marker_array/hca_back', MarkerArray, queue_size = 20)
    hcaSideArrayPub = rospy.Publisher('/visualization_marker_array/hca_side', MarkerArray, queue_size = 20)
    gapArrayPub = rospy.Publisher('/visualization_marker_array/gap', MarkerArray, queue_size = 20)
    
    objectArrayPub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 20)
    
    textArrayPub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size = 20)

    rate = rospy.Rate(freq)
    
    transformBroadcaster = TransformBroadcaster()
        

    while not rospy.is_shutdown():
        
        tt = time.time()
        if (tt - last_publish_time) > min_publish_dt:
            last_publish_time = tt 
            
            # Handling the publishing of detections
            if len(receivedDetections) > 0:
                
            
                activeClasses = []
                activeDetections = []
                corners = []
                centers = []
                quaternions = []
                #rospy.loginfo("vp_data run")
                
                if vision_lookup_topic == 'basler':
                    for detection in receivedDetections:
                        #rospy.loginfo("Got detection")
                        n = Label(detection.label).name
                        
                        activeClasses.append(n)
                        activeDetections.append(detection)
                        #centers.append(detection.tf.translation)
                        tr = copy.deepcopy(detection.tf.translation)   # Table should be rotated in config but that is a lot more work
                        tr.y = 0.6 - tr.y
                        tr.z = 0 # Hardcoded, but keep it this way. The HCAs are always on the table at height Z=0 m 
                        
                        
                        #rospy.loginfo(obb)
                        centers.append(tr)
                        
                        c1 = [detection.obb_3d[0], detection.obb_3d[1]]
                        c2 = [detection.obb_3d[3], detection.obb_3d[4]]
                        c3 = [detection.obb_3d[6], detection.obb_3d[7]]
                        c4 = [detection.obb_3d[10], detection.obb_3d[11]]

                        corners.append([c1,c2,c3, c4])
                        
                        obb = detection.obb_px
                        quat = obb_px_to_quat(obb)
                        quaternions.append(quat)
                        
                        #quaternions.append(detection.tf.rotation)
                        
                elif vision_lookup_topic == 'realsense':
                    for detection in receivedDetections:
                        
                        isvalid, detection = detectionChecker.checkDetection(detection)
                        
                        if isvalid:
                            #rospy.loginfo("Got detection")
                            activeClasses.append(Label(detection.label).name)
                            activeDetections.append(detection)
                            # Convert pixels to world coordinates.
                            X,Y,Z = uv_to_XY(u = detection.center_px[0], v = detection.center_px[1], P=P, Z = None, get_z_from_rosparam = True)
                            #rospy.loginfo("X: {}, Y: {}, Z: {}".format(X,Y,Z))
                            #rospy.loginfo(cameraTf)
                            tr = copy.deepcopy(detection.tf.translation)
                            
                            tr.x = X; tr.y = Y; tr.z = Z
                            centers.append(tr)
                            
                            # This can be done much nicer but i'm in a hurry and testing.
                            # Comment this out to not show markers for realsense detections ! 
                            X1,Y1,Z = uv_to_XY(u = detection.obb_px[0], v = detection.obb_px[1], P=P, Z = None, get_z_from_rosparam = True)
                            X2,Y2,Z = uv_to_XY(u = detection.obb_px[2], v = detection.obb_px[3], P=P, Z = None, get_z_from_rosparam = True)
                            X3,Y3,Z = uv_to_XY(u = detection.obb_px[4], v = detection.obb_px[5], P=P, Z = None, get_z_from_rosparam = True)
                            X4,Y4,Z = uv_to_XY(u = detection.obb_px[6], v = detection.obb_px[7], P=P, Z = None, get_z_from_rosparam = True)
                            corners.append([[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]])
                            
                            obb = detection.obb_px
                            #rospy.loginfo(obb)
                            quat = obb_px_to_quat(obb)
                            quaternions.append(quat)
                        
                    

                        #quaternions.append(detection.tf.rotation)
                try: 
                    activeFrames, activeColors = trackActiveFramesAndColors(activeClasses, activeDetections, name_addition)
                    sendTfTransform(transformBroadcaster, activeFrames, centers, quaternions, cameraTf)
                    
                    ttt = time.time()
                    if ttt- detectionTime > max_detection_age_seconds:
                        receivedDetections = []
                    
                    if len(corners)>0:
                        activeEdges = calculateObjectEdges(corners)

                        objectArray = prepareObjectArray(activeFrames, activeColors, activeEdges, cameraNs)
                        textArray = prepareTextArray(activeFrames, cameraNs)
                        batteryArray = filterMarkerArray(objectArray, outputClass = 'battery')
                        hcaFrontArray = filterMarkerArray(objectArray, outputClass = 'hca_front')
                        hcaBackArray =  filterMarkerArray(objectArray, outputClass = 'hca_back')
                        hcaSideArray = filterMarkerArray(objectArray, outputClass = 'hca_side')
                        objectArrayPub.publish(objectArray)
                        textArrayPub.publish(textArray)
                    
                        batteryArrayPub.publish(batteryArray)
                        hcaFrontArrayPub.publish(hcaFrontArray)
                        hcaBackArrayPub.publish(hcaBackArray)
                        hcaSideArrayPub.publish(hcaSideArray)    
                    
            
                
                except Exception as e:
                    rospy.loginfo("vp_data_visualization exception: {0}".format(e))
            
            
             # Handling the publishing of gaps
            if vision_lookup_topic == 'realsense':
               
                if len(receivedGaps) > 0:
                    activeFrames = []
                    centers = []
                    quaternions = []
                    
                    
                    for gap in receivedGaps:
                        
                        # Get the center from OBB and then calculate position in da world.
                        #p1 = gap.obb[0], gap.obb[1]
                        #p2 = gap.obb[4], gap.obb[5]
                        
                        center = (gap.obb[0] + gap.obb[4])/2 ,  (gap.obb[1] + gap.obb[5])/2
                        
                        X,Y,Z = uv_to_XY(u = center[0], v = center[1], P=P, Z = None, get_z_from_rosparam = True)
                        center_of_gap.x = X
                        center_of_gap.y = Y
                        center_of_gap.z = Z
                        
                        # Each gap has element from_tf and to_tf. We will take the POINT IN BETWEEN and draw it as a ros TF.
                        #x1 = gap.from_tf.translation
                        #x2 = gap.to_tf.translation
                        # Coordinates are swapped in both to_tf and from_tf messages. X is actually Z. Z is X.
                        #center_of_gap.z = (x1.x + x2.x)/2
                        #center_of_gap.x = (x1.y + x2.y)/2
                        #center_of_gap.y = (x1.z + x2.z)/2
                        
                        activeFrames.append('gap_%d_%s'%(gap.id, vision_lookup_topic))
                        centers.append(center_of_gap)
                        
                        rot = tf.transformations.quaternion_from_euler(np.pi, 0, 0) # Hardcoded, fix eet.
                        quaternions.append(rot)
                        #rospy.loginfo()
                        
                    # publishing the Gap markers
                    #objectArray = prepareObjectArray(activeFrames, activeColors, activeEdges, cameraNs)
                    #gapArray = filterMarkerArray(objectArray, outputClass = 'gap')
                    #gapArrayPub.publish(gapArray)
                    
                    sendTfTransform(transformBroadcaster, activeFrames, centers, quaternions, cameraTf)
                    
            
        
   
