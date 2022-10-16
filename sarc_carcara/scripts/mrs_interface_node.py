#!/usr/bin/env python

import rospy
from mrs_msgs.msg import SpeedTrackerCommand, Float64Stamped
from geometry_msgs.msg import Twist, Accel
from mrs_msgs.srv import String

import numpy as np

class MrsInterfaceNode:
    def __init__(self):
        self.ns = rospy.get_namespace().replace("/", "")

        self.command_published = False
        self.switched_tracker = False

        self.heading = 0
        self.last_msg_stamp = -1

        # read velocity from controller
        vel_sub = rospy.Subscriber("cmd_vel", Twist, callback=self.vel_cb)
        accel_sub = rospy.Subscriber("cmd_accel", Accel, callback=self.accel_cb)
        heading_sub = rospy.Subscriber("control_manager/heading", Float64Stamped, self.heading_cb)

        self.command_pub = rospy.Publisher("control_manager/speed_tracker/command", SpeedTrackerCommand, queue_size=10)
        rospy.wait_for_service("control_manager/switch_tracker", timeout=None)
        self.switch_tracker_srv = rospy.ServiceProxy("control_manager/switch_tracker", String)

    def heading_cb(self, msg: Float64Stamped):
        self.heading = msg.value

    def accel_cb(self, msg: Accel):
        accel = msg.linear
        command_msg = SpeedTrackerCommand()
        command_msg.header.stamp = rospy.Time.now()
        command_msg.header.frame_id = "{}/gps_origin".format(self.ns)
        command_msg.acceleration.x = accel.x
        command_msg.acceleration.y = accel.y
        command_msg.acceleration.z = accel.z
        command_msg.use_acceleration = True
        self.command_pub.publish(command_msg)

    def vel_cb(self, msg: Twist):
        vel = msg.linear
        command_msg = SpeedTrackerCommand()
        command_msg.header.stamp = rospy.Time.now()
        command_msg.header.frame_id = "{}/gps_origin".format(self.ns)
        command_msg.velocity.x = vel.x
        command_msg.velocity.y = vel.y
        command_msg.velocity.z = vel.z
        # command_msg.heading = np.arccos(vel.x/np.linalg.norm([vel.x, vel.y]))
        command_msg.use_velocity = True
        # command_msg.use_heading = True
        self.command_pub.publish(command_msg)
        self.command_published = True


rospy.init_node("mrs_interface_node")
node = MrsInterfaceNode()

rospy.sleep(15)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if node.command_published and not node.switched_tracker:
        try:
            resp = node.switch_tracker_srv("SpeedTracker")
            if resp.success:
                rospy.loginfo("Successfully switched to SpeedTracker")
                node.switched_tracker = True
            else:
                rospy.logwarn(resp.message)

        except rospy.ServiceException as e:
            rospy.logerr("Switch tracker service call failed: ".format(str(e)))

    rate.sleep()
