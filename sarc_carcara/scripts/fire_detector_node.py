#!/usr/bin/env python

import rospy
import image_geometry
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from sarc_carcara.msg import FireDetectionResult

from cv_bridge import CvBridge
import tf

import fire_detector

import numpy as np

class FireDetectorNode:
    def __init__(self) -> None:
        self.tf_listener = tf.TransformListener()
        self.camera_model = image_geometry.PinholeCameraModel()
        self.is_model_set = False

        self.bridge = CvBridge()

        self.fire_detector = fire_detector.FireDetector()
        
        camera_info_sub = rospy.Subscriber('bluefox_optflow/camera_info', CameraInfo, callback=self.camera_info_cb)
        image_sub = rospy.Subscriber('bluefox_optflow/image_raw', Image, self.image_cb)

        self.fire_pos_pub = rospy.Publisher('fire_detection_result', FireDetectionResult, queue_size=10) 

    def camera_info_cb(self, msg: CameraInfo):
        # set camera model parameters if not set
        if not self.is_model_set:
            self.camera_model.fromCameraInfo(msg)
            self.is_model_set = True

    def image_cb(self, msg: Image):
        if not self.is_model_set:
            rospy.loginfo('Waiting for camera info.')
        else:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            fire_center = self.fire_detector.locate_fire(img)
            
            result_msg = FireDetectionResult()

            if fire_center is None:
                result_msg.fire_detected = False
            else:
                rect = self.camera_model.rectifyPoint(fire_center)
                p_c = self.camera_model.projectPixelTo3dRay(rect) # point for z = 1 in the camera frame
                fire_pos_world = self.point_in_world(p_c)
                # rospy.loginfo("Fire detected at position ({}, {}, {})!".format(fire_pos_world[0], fire_pos_world[1], fire_pos_world[2]))
                result_msg.fire_detected = True
                result_msg.position = self.point_msg(fire_pos_world)
            
            self.fire_pos_pub.publish(result_msg)
    
    def point_in_world(self, p_c):
        pose_msg = self.pose_msg(p_c)
        ns = pose_msg.header.frame_id.split("/")[0]
        self.tf_listener.waitForTransform("{}/gps_origin".format(ns), pose_msg.header.frame_id, rospy.Time.now(), rospy.Duration(2.0))
        ray_world = self.pose_msg_to_array(self.tf_listener.transformPose("{}/gps_origin".format(ns), pose_msg)) - np.array([0.0, 0.0, 5.0]) # shift z-axis to consider tree height
        
        self.tf_listener.waitForTransform("{}/gps_origin".format(ns), pose_msg.header.frame_id, rospy.Time.now(), rospy.Duration(2.0))
        (camera_pos, _) = self.tf_listener.lookupTransform("{}/gps_origin".format(ns), self.camera_model.tfFrame(), rospy.Time(0))
        camera_pos = np.array(camera_pos) - np.array([0.0, 0.0, 5.0]) # shift z-axis to consider tree height

        p = ray_world - camera_pos
        theta = np.arcsin(np.sqrt(p[0]**2 + p[1]**2)/np.linalg.norm(p))

        pos_fire_length = np.abs(camera_pos[2]/np.cos(theta))

        pos_fire = pos_fire_length*(np.array(p)/np.linalg.norm(p)) + camera_pos
        return pos_fire

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

rospy.init_node('fire_detector_node')
node = FireDetectorNode()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
