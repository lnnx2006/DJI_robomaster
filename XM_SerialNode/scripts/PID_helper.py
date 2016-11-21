#!/usr/bin/python
"""
Created on  July 27 2014

"""

import rospy
from XM_msgs.msg import XM_Datagram
from std_msgs.msg import Float32
import struct

def callback(data):
    set_point, motor, encoder,  = struct.unpack("!hhh", data.data)
    pub1.publish(Float32(set_point))
    pub2.publish(Float32(motor))
    pub3.publish(Float32(encoder))

if __name__ == '__main__':
    rospy.init_node('PID_publisher')
    sub = rospy.Subscriber('RecvData/105', XM_Datagram, callback)
    pub1 = rospy.Publisher('angle', Float32)
    pub2 = rospy.Publisher('target', Float32)
    pub3 = rospy.Publisher('position', Float32)
    rospy.spin()
