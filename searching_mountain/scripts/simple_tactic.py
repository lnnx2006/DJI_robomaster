#!/usr/bin/env python
'''
Created on 2014-8-15

'''
'''
used to send the navi goal for move_base according to the received enemy data and current position
'''
import rospy
from can_msg.msg import *
from smach import *
from smach_ros import *
from nav_msgs.msg import Odometry
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction


class Nav_next_point(State):
    def __init__(self):
        State.__init__(self, outcomes=['done', 'preepted'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.get_scored_goal = rospy.ServiceProxy('get_scored_goal', )
        self.goal_sent = False

    def execute(self, ud):
        self.client.wait_for_server()
        #new_goal = self.get_scored_goal.call()
        self.client.send_goal(new_goal)
        self.client.wait_for_result()
        self.goal_sent = True

    def preempt_requested(self):
        State.request_preempt(self)
        print "be preempted"

    def service_preempt(self):
        State.service_preempt()


class SimpleTatic:
    def __init__(self):
        rospy.init_node("single_tactic",anonymous=True)
        #parameter setting
        self.safe_blood = rospy.get_param("safe_blood", 500)
        self.safe_dist = rospy.get_param("safe_dist", 3.0)
        self.aggre_dist = rospy.get_param("aggre_dist", 1.5)
        #input data








if __name__ == "__main__":


    rospy.spin()