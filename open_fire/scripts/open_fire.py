#!/usr/bin/python
"""
Created on  Aug 1 2014

"""

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from can_msg.msg import *
import math

class Open_fire:

    def odom_CB(self, data):
        self.current_odom = data
        self.odom_update = True
        return

    def gimbal_CB(self, data):
        self.current_state = data
        self.joint_update = True
        return

    def safety_CB(self, data):
        self.safety = data.safe


    def check_state_CB(self):
        if self.joint_update and self.odom_update and self.safety:
            #check the gun locked or not
            if self.current_state.yaw < self.bias and self.current_state.pitch < self.bias and \
                self.current_odom.twist.twist.linear.x + self.current_odom.twist.twist.linear.y + self.current_odom.twist.twist.angular.z < self.velocity_bias and \
                rospy.get_time() - self.last_shot > self.fire_interval:
                fire_ = fire()
                print "firing"
                fire_.fire = True
                fire_.header.stamp = rospy.Time.now()
                counter = 40
                rate = rospy.Rate(30)
                self.fire_status.publish(fire_)
                while counter > 0:
                    self.cmd_fire_pub.publish(fire_)
                    counter -= 1
                    rate.sleep()
                self.last_shot = rospy.get_time()
                self.joint_update = False
                self.odom_update = False
        else:
            fire_ = fire()
            fire_.fire = False
            fire_.header.stamp = rospy.Time.now()
            self.fire_status.publish(fire_)


    def __init__(self):
        #publisher and subscriber
        rospy.init_node('open_fire', anonymous=True)
        self.cmd_fire_pub = rospy.Publisher('tank102/cmd_fire', fire)
        self.fire_status = rospy.Publisher('tank102/fire_status', fire)
        self.odom_sub = rospy.Subscriber('tank102/odom', Odometry, self.odom_CB)
        self.gimbal_sub = rospy.Subscriber('tank102/cmd_gimbal', gimbal, self.gimbal_CB)
        self.safety_sub = rospy.Subscriber('tank102/safety', safety, self.safety_CB)
        self.odom_update = False
        self.joint_update = False
        #param
        self.bias = rospy.get_param('locked_bias', 30)
        self.fire_interval = rospy.get_param('fire_interval', 1.0)
        self.check_rate = rospy.get_param('fire_check_rate', 5)
        self.velocity_bias = rospy.get_param('velocity_bias', 1000)
        self.current_state = gimbal()
        self.current_odom = Odometry()
        self.last_shot = rospy.get_time()
        self.safety = True # open fire permission

    def mainloop(self):
        rate = rospy.Rate(self.check_rate)
        while not rospy.is_shutdown():
            self.check_state_CB()
            rate.sleep()
        rospy.spin()

if __name__ == "__main__":
    open_fire = Open_fire()
    open_fire.mainloop()

