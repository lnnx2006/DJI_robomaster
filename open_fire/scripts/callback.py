#!/usr/bin/python
"""
Created on  Aug 1 2014

"""

import rospy
def my_callback(event):
    print 'Timer called at ' + str(event.current_real)

if __name__ == '__main__':
    rospy.init_node('timer')

    rospy.Timer(rospy.Duration(2), my_callback)
    rospy.spin()