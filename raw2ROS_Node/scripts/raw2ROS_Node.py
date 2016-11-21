#!/usr/bin/python
"""
Created on  July 27 2014

"""
#used for receive other nodes' data and broadcast them in the CAN bus
import rospy
import tf

from can_msg.msg import *
from XM_msgs.msg import XM_Datagram
from std_msgs.msg import *
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import threading
import struct

ini_data = [0xff]
odom_data = [0xf1]
fire_data = [0x11]
wheel_radius = 0.075
encoder_line = 8000
increment_X = 0.09
increment_Y = 0.08
# used to calculate the odometry
w1_cyc = w1_line = w2_cyc = w2_line = w3_cyc = w3_line = w4_cyc = w4_line = Th = 0
w1_cyc_p = w1_line_p = w2_cyc_p = w2_line_p = w3_cyc_p = w3_line_p = w4_cyc_p = w4_line_p = preTh = 0
now_stamp_odom = 0.0
past_stamp_odom = 0.0
now_odom = Odometry()
past_odom = Odometry()
# used to calculate the yaw and pitch
yaw = pitch = 0
yaw_p = pitch_p = 0
now_stamp_gim = 0.0
past_stamp_gim = 0.0
# check the callback execute or not
updated = 0
raw_data = XM_Datagram()

class Can_Node:

    def calc_pub_inverse_odom(self, data):
        print "odom arrived"
        #cycle num and line num ,distance = (cyc * encoder_line + line) / encoder_line * wheel_radius * math.pi
        global now_stamp_odom, past_stamp_odom, now_odom, past_odom, updated
        global w1_cyc, w1_line, w2_cyc, w2_line, w3_cyc, w3_line, w4_cyc, w4_line, Th, w1_cyc_p, w1_line_p, w2_cyc_p, w2_line_p, w3_cyc_p, w3_line_p, w4_cyc_p, w4_line_p, preTh
        w1_cyc, w1_line, w2_cyc, w2_line, w3_cyc, w3_line, w4_cyc, w4_line, = struct.unpack('!hhhhhhhh', data.data)
        now_stamp_odom = rospy.get_time()
        dT = now_stamp_odom - past_stamp_odom
        if dT > (1.5 / self.odom_rate):
            print "Odom data delay a lot, please check the rate or serial port"
        dw1_dis = ((float)((w1_cyc - w1_cyc_p) * encoder_line + w1_line - w1_line_p) / encoder_line * 2 * math.pi) / dT
        dw2_dis = ((float)((w2_cyc - w2_cyc_p) * encoder_line + w2_line - w2_line_p) / encoder_line * 2 * math.pi) / dT
        dw3_dis = ((float)((w3_cyc - w3_cyc_p) * encoder_line + w3_line - w3_line_p) / encoder_line * 2 * math.pi) / dT
        dw4_dis = ((float)((w4_cyc - w4_cyc_p) * encoder_line + w4_line - w4_line_p) / encoder_line * 2 * math.pi) / dT

        w1_cyc_p = w1_cyc
        w2_cyc_p = w2_cyc
        w3_cyc_p = w3_cyc
        w4_cyc_p = w4_cyc
        w1_line_p = w1_line
        w2_line_p = w2_line
        w3_line_p = w3_line
        w4_line_p = w4_line

        #print dw1_dis , "   ", dw2_dis , "   ", dw3_dis , "   ", dw4_dis , "   " , preTh,"   ", dT
        now_odom.header.frame_id = 'odom'
        now_odom.header.stamp = rospy.Time.now()
        now_odom.child_frame_id ="base_link"
        now_odom.twist.twist.linear.x =  dw1_dis     * 0.01875 + dw2_dis * 0.01875 + dw3_dis * 0.01875 + dw4_dis * 0.01875
        now_odom.twist.twist.linear.y =  (- dw1_dis) * 0.01875 + dw2_dis * 0.01875 + dw3_dis * 0.01875 - dw4_dis * 0.01875
        now_odom.twist.twist.angular.z = dw1_dis     * -0.0436  + dw2_dis * 0.0436   + dw3_dis * -0.0436  + dw4_dis * 0.0436
        Th = preTh + now_odom.twist.twist.angular.z * dT

        now_odom.pose.pose.position.x = past_odom.pose.pose.position.x + (now_odom.twist.twist.linear.x * math.cos(preTh) - now_odom.twist.twist.linear.y * math.sin(preTh)) * dT
        now_odom.pose.pose.position.y = past_odom.pose.pose.position.y + (now_odom.twist.twist.linear.x * math.sin(preTh) + now_odom.twist.twist.linear.y * math.cos(preTh)) * dT
        now_odom.pose.pose.orientation.z = math.sin(Th / 2)
        now_odom.pose.pose.orientation.w = math.cos(Th / 2)
        preTh = Th
        #print (now_odom.twist.twist.linear.x * math.cos(preTh) - now_odom.twist.twist.linear.y * math.sin(preTh)) * dT, "    ",(now_odom.twist.twist.linear.x * math.sin(preTh) + now_odom.twist.twist.linear.y * math.cos(preTh)) * dT ,"    ", preTh
        self.odom_pub.publish(now_odom)
        #print now_odom
        past_stamp_odom = rospy.get_time()
        past_odom = now_odom
        self.tf_broadcaster.sendTransform((now_odom.pose.pose.position.x, now_odom.pose.pose.position.y, 0), (now_odom.pose.pose.orientation.x, now_odom.pose.pose.orientation.y, now_odom.pose.pose.orientation.z, now_odom.pose.pose.orientation.w), rospy.Time.now(), 'base_link', '/odom')
        return

    def calc_pub_gimbal(self, data):
        print "gimbal data received"
        global now_stamp_gim, past_stamp_gim, yaw_p, yaw, pitch, pitch_p, updated
        joint_states = JointState()
        w6_line, w5_line, = struct.unpack('!hh', data.data)
        dT = now_stamp_gim - past_stamp_gim
        if dT - (1.5 / self.odom_rate) > 0:
            print "Odom data delay a lot, please check the rate or serial port"
        yaw = -w5_line * math.pi * 2 / 360 * 0.044
        pitch = -(w6_line) * 2 * math.pi / 360 * 0.044
        #construct a joint_states for publish
        joint_states.header.stamp = rospy.Time.now()
        joint_states.name.append('joint_waist')
        joint_states.position.append(yaw)
        joint_states.velocity.append((yaw - yaw_p) / dT)
        joint_states.name.append('joint_shoulder')
        joint_states.position.append(pitch)
        joint_states.velocity.append((pitch - pitch_p) / dT)
        #update the temp var
        yaw_p = yaw
        pitch_p = pitch
        self.joint_state_pub.publish(joint_states)
        past_stamp_gim = rospy.get_time()

    def cmd_vel_CB(self,data):
        #calculate the forward kinematics and send the velocity to the base
        global updated
        if updated == 0:
            self.send_datagram(self.PC_base_ID, self.base_ID, [0xf2])
            print "dsgga"
            updated = 1
        rw1 = (data.linear.x * 13.3333 + data.linear.y * (-13.3333) + data.angular.z * (-5.7333)) / 100.0
        rw2 = (data.linear.x * 13.3333 + data.linear.y * 13.3333    + data.angular.z * 5.7333) / 100.0
        rw3 = (data.linear.x * 13.3333 + data.linear.y * 13.3333    + data.angular.z * (-5.7333)) / 100.0
        rw4 = (data.linear.x * 13.3333 + data.linear.y * (-13.3333) + data.angular.z * 5.73330) / 100.0
        rw1_line = (rw1 / math.pi / 2 * encoder_line)
        rw2_line = (rw2 / math.pi / 2 * encoder_line)
        rw3_line = (rw3 / math.pi / 2 * encoder_line)
        rw4_line = (rw4 / math.pi / 2 * encoder_line)
        data1 = struct.pack('!hh', rw1_line, rw2_line)
        data2 = struct.pack('!hh', rw3_line, rw4_line)
        self.send_datagram(self.PC_base_ID, self.base_ID, data1 + data2)

    def cmd_gimbal_CB(self, data):
        global yaw, pitch, increment_X, increment_Y
	if data.yaw == 999.0 and data.pitch == 999.0: 
	    data_ = struct.pack('!hh', 999, 999)
            self.send_datagram(self.PC_gimbal_ID, self.gimbal_ID, data_)
            print data_
            return 
        #rw5_line = -int(((data.yaw * increment_X + yaw)*57.3 ) / 0.044)
        #rw6_line = -int(((data.pitch * increment_Y + pitch)*57.3 ) / 0.044)
        rw5_line = int(data.yaw)
        rw6_line = int(data.pitch)
        #print "yaw:", data.yaw, "  ", (data.yaw * increment_X + yaw)*57.3 ,"  ",rw5_line ,"    ",yaw
        #print "pitch:", data.pitch, "  ", (data.pitch * increment_Y + pitch)*57.3 ,"  ",rw6_line ,"    ",pitch
        data_ = struct.pack('!hh', rw6_line, rw5_line)
        print rw6_line, rw5_line
        self.send_datagram(self.PC_gimbal_ID, self.gimbal_ID, data_)

    def cmd_fire_CB(self,data):
        if data.fire:
            self.send_datagram(self.PC_gimbal_ID, self.gimbal_ID, fire_data)
        else:
            return

    def send_datagram(self, sender, receiver, data):
        with self.lock:
            datagram = XM_Datagram()
            datagram.data = data
            datagram.sender = sender
            datagram.receiver = receiver
            self.serial_pub.publish(datagram)

    def ask_serial_data(self):
        print "ask for serial data"
        self.send_datagram(self.PC_base_ID, self.base_ID, odom_data)
        self.send_datagram(self.PC_gimbal_ID, self.gimbal_ID, odom_data)

    def __init__(self):
        #set the IDs and some params
        rospy.init_node('raw2ROS_node', anonymous=True)
        self.base_ID = rospy.get_param('base_ID', 0x05)
        self.gimbal_ID = rospy.get_param('gimbal_ID', 0x07)
        self.PC_base_ID = rospy.get_param('PC_base_ID', 0x44)
        self.PC_gimbal_ID = rospy.get_param('PC_gimbal_ID', 0x46)
        self.odom_rate = rospy.get_param('Odom_rate', 15)

        #rospy sub and pub init
        self.base_sub = rospy.Subscriber('RecvData/68', XM_Datagram, self.calc_pub_inverse_odom)
        self.gimbal_sub = rospy.Subscriber('RecvData/70' , XM_Datagram, self.calc_pub_gimbal)
        self.odom_pub = rospy.Publisher('odom', Odometry)
        self.serial_pub = rospy.Publisher('SendSerialData', XM_Datagram)
        self.joint_state_pub = rospy.Publisher('joint_states', JointState)
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_CB)
        self.gimbal_sub = rospy.Subscriber('cmd_gimbal', gimbal, self.cmd_gimbal_CB)
        self.fire_cmd = rospy.Subscriber('cmd_fire', fire, self.cmd_fire_CB)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.lock = threading.Lock()

    def mainloop(self):
        #inialize the base
        self.send_datagram(self.PC_base_ID, self.base_ID, ini_data)
        #inialize the gimbal
        #self.send_datagram(self.PC_ID, self.gimbal_ID, ini_data)
        #ask for coder data
        global past_stamp_gim, past_stamp_odom
        past_stamp_odom = rospy.get_time()
        past_stamp_gim = rospy.get_time()
        rate = rospy.Rate(self.odom_rate)
        global w1_cyc, w1_line, w2_cyc, w2_line, w3_cyc, w3_line, w4_cyc, w4_line, Th, w1_cyc_p, w1_line_p, w2_cyc_p, w2_line_p, w3_cyc_p, w3_line_p, w4_cyc_p, w4_line_p, preTh
        w1_cyc = w1_line = w2_cyc = w2_line = w3_cyc = w3_line = w4_cyc = w4_line = Th = 0
        w1_cyc_p = w1_line_p = w2_cyc_p = w2_line_p = w3_cyc_p = w3_line_p = w4_cyc_p = w4_line_p = preTh = 0
        while not rospy.is_shutdown():
            self.ask_serial_data()
            rate.sleep()

if __name__ == "__main__":
    can_node = Can_Node()
    can_node.mainloop()
    rospy.spin()
