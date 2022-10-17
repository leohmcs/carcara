#!/usr/bin/python3

import sys

import rospy
import rosnode
import math

from mrs_msgs.msg import UavManagerDiagnostics as UavManagerDiagnostics
from mavros_msgs.srv import CommandBool as CommandBool
from std_srvs.srv import Trigger as Trigger
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest, ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3

# global uav_name
# uav_name = "uav1"
class Activator:
    def __init__(self, uav_id: int):
        self.uav_name = "uav" + str(uav_id)

        rospy.init_node('activator', anonymous=True) 

        self.delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)    
        self.subscriber = rospy.Subscriber('/' + self.uav_name + '/uav_manager/diagnostics', UavManagerDiagnostics, self.callback)
        self.arm = rospy.ServiceProxy('/' + self.uav_name + '/mavros/cmd/arming', CommandBool)
        self.activate = rospy.ServiceProxy('/' + self.uav_name + '/uav_manager/midair_activation', Trigger)
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        
        rospy.loginfo('initialized')
        rospy.spin()

    def callback(self, data):
        body_name = self.uav_name + '::base_link'
        wrench = Wrench()
        force = [1, 1, 1]
        wrench.force = Vector3(*force)
        duration = rospy.Duration(100)
        
        dele = DeleteModelRequest()
        dele.model_name = "SARckc_floor"
        self.delete(dele)
        rospy.loginfo('arming')
        self.arm(1)
        rospy.sleep(5)
        rospy.loginfo('activating')
        self.activate()
        rospy.sleep(8)
        rospy.loginfo('applying')
        self.apply_wrench(body_name, 'world', Point(0, 0, 0), wrench, rospy.Time().now(), duration)
        rospy.signal_shutdown("yes")

if __name__ == '__main__':
    try:
        uav_id = int(sys.argv[1])
        node_crash_checker = Activator(uav_id)
    except rospy.ROSInterruptException:
        pass
