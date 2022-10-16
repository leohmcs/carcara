#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from robotsim.msg import Vision_msg

'''This emulates the sensor'''


class Vision:

    def __init__(self):


        # Create node, publisher e subscriber
        rospy.init_node(f"vision_node")

        self.robot_number = rospy.get_param('~robot_number')

        self.state_subscriber = rospy.Subscriber(f"trajectory_state",  Float32, self.trajectory_state_cb)
        self.pub_vision_state = rospy.Publisher(f"vision_state",  Vision_msg, queue_size=100)
        self.vision_msg =  Vision_msg()

        self.rate = rospy.Rate(10)
        self.aux = 0
        self.rate.sleep()
        self.trajectory_state = 0

    def trajectory_state_cb(self,data):
        self.trajectory_state = data

    

    def run(self):
        
        while not rospy.is_shutdown():

            if self.trajectory_state == 0:
                pass

            elif self.trajectory_state.data == 5:
                rospy.sleep(80)
                self.vision_msg.saw_it = 1
                self.vision_msg.x = -2
                self.vision_msg.y = -3
                self.vision_msg.z = 2
                self.vision_msg.fire_extinguished = 1
                self.pub_vision_state.publish(self.vision_msg)

            elif (self.aux == 0 and self.trajectory_state != 0):
                self.aux += 1 
                dados = [0,-2,-3,2,0]
                self.vision_msg.saw_it = 0
                self.vision_msg.x = -2
                self.vision_msg.y = -3
                self.vision_msg.z = 2
                self.vision_msg.fire_extinguished = 0
                self.pub_vision_state.publish(self.vision_msg)
                rospy.sleep(40)

            elif(self.aux == 1):
                dados = [1,-2,-3,2,0]
                self.vision_msg.saw_it = 1
                self.vision_msg.x = -2
                self.vision_msg.y = -3
                self.vision_msg.z = 2
                self.vision_msg.fire_extinguished = 0
                self.pub_vision_state.publish(self.vision_msg)

            else:
                print("Something went wrong in vision node")
                pass
        



    

# Main function
if __name__ == '__main__':

    try:
        vis = Vision()
        vis.run()
    except rospy.ROSInterruptException:
        pass

    rospy.sleep(.5)