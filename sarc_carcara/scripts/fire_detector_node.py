#!/usr/bin/env python

import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sarc_carcara.msg import FireDetectionResult
from std_msgs.msg import Bool

from cv_bridge import CvBridge
import tf

import fire_detector

import numpy as np

class FireDetectorNode:
    def __init__(self) -> None:
        self.pos = None
        self.quat = None
        self.tree_height = 7

        self.ns = rospy.get_namespace().replace('/', '')

        self.tf_listener = tf.TransformListener()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.is_model_set = False

        self.bridge = CvBridge()

        self.fire_detector = fire_detector.FireDetector()
        
        camera_info_sub = rospy.Subscriber('bluefox_optflow/camera_info', CameraInfo, callback=self.camera_info_cb)
        image_sub = rospy.Subscriber('bluefox_optflow/image_raw', Image, self.image_cb)
        odom_sub = rospy.Subscriber('odometry/odom_gps', Odometry, self.odom_cb)
        # disarm_sub = rospy.Subscriber('disarm', Bool, self.disarm_cb)

        self.fire_pos_pub = rospy.Publisher('fire_detection_result', FireDetectionResult, queue_size=10) 

    def camera_info_cb(self, msg: CameraInfo):
        # set camera model parameters if not set
        if not self.is_model_set:
            self.camera_model.fromCameraInfo(msg)
            self.is_model_set = True

    def image_cb(self, msg: Image):
        if not self.is_model_set:
            self.log('Waiting for camera info.', rospy.loginfo)
        else:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            fvert = self.fire_detector.locate_fire(img)     # fire vertices
            
            result_msg = FireDetectionResult()

            if self.pos is None or self.quat is None:
                rospy.loginfo('[{}] Odom not received.'.format(self.ns))
                return
                
            if fvert is None or self.pos[2] < 1.0:
                result_msg.fire_detected = False
            else:
                # diag = self.estimate_fire_diagonal(fire_vertices)

                mid_x = int((fvert[0] + fvert[2])/2)
                mid_y = int((fvert[1] + fvert[3])/2)
                # fire_center = [mid_x, mid_y]
                poi = [[mid_x, mid_y], [fvert[0], fvert[1]], [fvert[0], fvert[3]], [fvert[2], fvert[3]], [fvert[2], fvert[1]]]    # points of interest
                poi_world = []
                for p in poi:
                    rect = self.camera_model.rectifyPoint(p)
                    p_c = self.camera_model.projectPixelTo3dRay(rect)       # point for z = 1 in the camera frame
                    pos_world = self.point_in_world(p_c)
                    poi_world.append(pos_world)

                # rospy.loginfo("Fire detected at position {}!".format(poi_world[o]))

                diags = np.abs([poi_world[0] - poi_world[2], poi_world[1] - poi_world[3]])
                result_msg.fire_detected = True
                result_msg.position = self.point_msg(poi_world[0])
                result_msg.largest_diag = np.max([diags])
            
            self.fire_pos_pub.publish(result_msg)
    
    def odom_cb(self, msg: Odometry):
        self.pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    
    # def disarm_cb(self, msg: Bool):
    #     if msg.data:
    #         return

    def point_in_world(self, p_c):
        pose_msg = self.pose_msg(p_c)
        ns = pose_msg.header.frame_id.split("/")[0]
        self.tf_listener.waitForTransform("{}/gps_origin".format(ns), pose_msg.header.frame_id, rospy.Time.now(), rospy.Duration(2.0))
        ray_world = self.pose_msg_to_array(self.tf_listener.transformPose("{}/gps_origin".format(ns), pose_msg))
        
        self.tf_listener.waitForTransform("{}/gps_origin".format(ns), pose_msg.header.frame_id, rospy.Time.now(), rospy.Duration(2.0))
        (camera_pos, _) = self.tf_listener.lookupTransform("{}/gps_origin".format(ns), self.camera_model.tfFrame(), rospy.Time(0))
        camera_pos = np.array(camera_pos)

        p = ray_world - camera_pos
        theta = np.arcsin(np.sqrt(p[0]**2 + p[1]**2)/np.linalg.norm(p))

        pos_fire_length = np.abs((camera_pos[2] - self.tree_height)/np.cos(theta))     # shift z-axis to consider tree height

        pos_fire = pos_fire_length*(np.array(p)/np.linalg.norm(p)) + camera_pos
        return pos_fire

    # def estimate_fire_diagonal(self, fire_vertices):
    #     ''' Return length of largest diagonal'''
    #     p1 = self.point_in_world([fire_vertices])

    def set_model(self):
        self.camera_model.fromCameraInfo(self.camera_left_info, self.camera_right_info)

    def point_msg(self, p):
        msg = Point()
        # msg.header.frame_id = 'world'
        # msg.header.stamp = rospy.Time.now()
        msg.x = p[0]
        msg.y = p[1]
        msg.z = p[2]
        return msg

    def pose_msg(self, p):
        msg = PoseStamped()
        msg.header.frame_id = self.camera_model.tfFrame()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = p[0]
        msg.pose.position.y = p[1]
        msg.pose.position.z = p[2]
        msg.pose.orientation.x = 0
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        msg.pose.orientation.w = 0
        return msg
    
    def pose_msg_to_array(self, msg: PoseStamped):
        pos = msg.pose.position
        return np.array([pos.x, pos.y, pos.z])

    def log(self, msg, f):
        msg = "[{}] ".format(self.ns) + msg
        f(msg)

rospy.init_node('fire_detector_node')
node = FireDetectorNode()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
