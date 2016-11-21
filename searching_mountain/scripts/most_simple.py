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
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
import math
import random
from geometry_msgs.msg import *

now_state = 0

def searching_term_cb(outcome_map):
    if outcome_map['nav_goal'] == 'done':
        return True
    elif outcome_map['enemy_detect'] == 'invalid':
        return True
    else :
        return False

def searching_out_cb(outcome_map):
    if outcome_map['enemy_detect'] == 'invalid':
        return 'engaging'
    if outcome_map['nav_goal'] == 'done':
        return 'searching'

def enemy_detect_cb_s(ud, msg):
    if not msg.safe:
        global now_state
        now_state = 0
        return True
    else:
        global now_state
        now_state = 1
        return False

def enemy_detect_cb_e(ud, msg):
    if  not msg.safe:
        global now_state
        now_state = 0
        return False
    else:
        global now_state
        now_state = 1
        return True

def engaging_term_cb(outcome_map):
    if outcome_map['nav_goal'] == 'done':
        return True
    elif outcome_map['enemy_detect'] == 'invalid':
        return True
    else:
        return False

def engaging_out_cb(outcome_map):
    if outcome_map['enemy_detect'] == 'invalid':
        return 'searching'
    if outcome_map['nav_goal'] == 'done':
        return 'engaging'


def get_new_searching_point():
        searching_points = []
        searching_point1 = MoveBaseGoal()
        searching_point1.target_pose.header.frame_id = 'map'
        searching_point1.target_pose.header.stamp = rospy.Time.now()
        searching_point1.target_pose.pose.position.x = 70.25
        searching_point1.target_pose.pose.position.y = 7.065
        searching_point1.target_pose.pose.orientation.w = 1.0
        searching_points.append(searching_point1)

        searching_point2 = MoveBaseGoal()
        searching_point2.target_pose.header.frame_id = 'map'
        searching_point2.target_pose.header.stamp = rospy.Time.now()
        searching_point2.target_pose.pose.position.x = 70.134
        searching_point2.target_pose.pose.position.y = -1.716
        searching_point2.target_pose.pose.orientation.w = 1.0
        searching_points.append(searching_point2)

        searching_point3 = MoveBaseGoal()
        searching_point3.target_pose.header.frame_id = 'map'
        searching_point3.target_pose.header.stamp = rospy.Time.now()
        searching_point3.target_pose.pose.position.x = 66.866
        searching_point3.target_pose.pose.position.y = 0.052
        searching_point3.target_pose.pose.orientation.w = 1.0
        searching_points.append(searching_point3)

        searching_point4 = MoveBaseGoal()
        searching_point4.target_pose.header.frame_id = 'map'
        searching_point4.target_pose.header.stamp = rospy.Time.now()
        searching_point4.target_pose.pose.position.x = 67.639
        searching_point4.target_pose.pose.position.y = 5.962
        searching_point4.target_pose.pose.orientation.w = 1.0
        searching_points.append(searching_point4)



        searching_point5 = MoveBaseGoal()
        searching_point5.target_pose.header.frame_id = 'map'
        searching_point5.target_pose.header.stamp = rospy.Time.now()
        searching_point5.target_pose.pose.position.x = 61.2
        searching_point5.target_pose.pose.position.y = 5.44
        searching_point5.target_pose.pose.orientation.w = 1.0
        searching_points.append(searching_point5)

        searching_point6 = MoveBaseGoal()
        searching_point6.target_pose.header.frame_id = 'map'
        searching_point6.target_pose.header.stamp = rospy.Time.now()
        searching_point6.target_pose.pose.position.x = 61.81
        searching_point6.target_pose.pose.position.y = -0.11
        searching_point6.target_pose.pose.orientation.w = 1.0
        searching_points.append(searching_point6)

        searching_point7 = MoveBaseGoal()
        searching_point7.target_pose.header.frame_id = 'map'
        searching_point7.target_pose.header.stamp = rospy.Time.now()
        searching_point7.target_pose.pose.position.x = 58.17
        searching_point7.target_pose.pose.position.y = 0.86
        searching_point7.target_pose.pose.orientation.w = 1.0
        searching_points.append(searching_point7)

        searching_point8 = MoveBaseGoal()
        searching_point8.target_pose.header.frame_id = 'map'
        searching_point8.target_pose.header.stamp = rospy.Time.now()
        searching_point8.target_pose.pose.position.x = 57.82
        searching_point8.target_pose.pose.position.y = 3.88
        searching_point8.target_pose.pose.orientation.w = 1.0
        searching_points.append(searching_point8)

        success = False
        trans = []
        rot = []
        listener = tf.TransformListener()
        print "dsggsdf"
        while not success:
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                success = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                success = False

        print trans , rot
        if now_state ==0 :
            '''find nearest point '''
            # nearest_point = MoveBaseGoal()
            # nearest_dis = 100000.0
            # for point in searching_points :
            #     if math.hypot(point.target_pose.pose.position.x - trans[0], point.target_pose.pose.position.y - trans[1]) > 0.6:
            #         nearest_point = point
            #         nearest_dis = math.hypot(point.target_pose.pose.position.x - trans[0], point.target_pose.pose.position.y - trans[1])
            # nearest_point.target_pose.header.stamp = rospy.Time.now()
            # nearest_point.target_pose.header.frame_id = 'map'
            # nearest_point.target_pose.pose.orientation.z = rot[2]
            # nearest_point.target_pose.pose.orientation.w = rot[3]
            # print nearest_point
            # return nearest_point
            '''find a random point to search'''
            get = False
            random_point = MoveBaseGoal()
            while not get:
                random_point = searching_points[random.randint(0, searching_points.__len__() - 1)]
                if math.hypot(random_point.target_pose.pose.position.x - trans[0], random_point.target_pose.pose.position.y - trans[1]) > 0.6 and math.hypot(random_point.target_pose.pose.position.x - trans[0], random_point.target_pose.pose.position.y - trans[1]) < 10.0:
                    get = True
            random_point.target_pose.header.stamp = rospy.Time.now()
            random_point.target_pose.header.frame_id = 'map'
            random_point.target_pose.pose.orientation.z = rot[2]
            random_point.target_pose.pose.orientation.w = rot[3]
            print random_point
            return random_point
        else:
            target = MoveBaseGoal()
            target.target_pose.header.stamp = rospy.Time.now()
            target.target_pose.header.frame_id = 'base_link'
            target.target_pose.pose.position.x = 1.0
            target.target_pose.pose.orientation.w = 1.0
            return target

class Nav_next_point(State):
    def __init__(self):
        State.__init__(self, outcomes=['done', 'preempted'])
        self.client = actionlib.SimpleActionClient('/tank102/move_base', MoveBaseAction)
        self.goal_sent = False

    def execute(self, ud):
        global now_state
        if now_state != 0 :
            rospy.sleep(3)
        print "waiting for server"
        self.client.wait_for_server()
        now_point = get_new_searching_point()
        self.goal_sent = True
        print "sending goal"
        print now_point
        self.client.send_goal(now_point)
        print "my status", self.client.get_state()
        while self.client.get_state() == 1 or self.client.get_state() == 0:# the state is active,  goal is running
            if self.preempt_requested():
                print "now goal is being preempted!!!"
                self.service_preempt()
                if self.goal_sent:
                    print "goal canceled"
                    self.client.cancel_goal()
                return 'preempted'
        # self.client.wait_for_result()
        return 'done'

class Simplest:
    def __init__(self):
        rospy.init_node('Simplest')
        # ini_pub = rospy.Publisher('/tank103/initialpose',PoseWithCovarianceStamped)
        # ini = PoseWithCovarianceStamped()
        # ini.header.frame_id = 'map'
        # ini.pose.pose.position.x = 73.04
        # ini.pose.pose.position.y = 5.733
        # ini.pose.pose.orientation.z = -1
        # ini.pose.covariance[0] = 0.25
        # ini.pose.covariance[7] = 0.25
        # ini.pose.covariance[35] = 0.068
        # ini_pub.publish(ini)
        # print ini

    def mainloop(self):
        global  now_state
        now_state = 0
        Searching_moutain = Concurrence(outcomes=['searching', 'engaging'],default_outcome='searching', child_termination_cb=searching_term_cb, outcome_cb=searching_out_cb)
        with Searching_moutain:
            Concurrence.add('nav_goal', Nav_next_point())
            Concurrence.add('enemy_detect', MonitorState("/tank102/safety", safety, enemy_detect_cb_s))
        Engaging = Concurrence(outcomes=['searching', 'engaging'],default_outcome='searching', child_termination_cb=engaging_term_cb, outcome_cb=engaging_out_cb)
        with Engaging:
            Concurrence.add('nav_goal', Nav_next_point())
            Concurrence.add('enemy_detect', MonitorState("/tank102/safety", safety, enemy_detect_cb_e))
        sm = StateMachine(outcomes=['ALL_DONE'])
        with sm:
            StateMachine.add('SEARCHING', Searching_moutain, transitions={'searching': 'SEARCHING', 'engaging': 'ENGAGING'})
            StateMachine.add('ENGAGING', Engaging, transitions={'searching': 'SEARCHING', 'engaging': 'ENGAGING'})
        sm.execute()

if __name__ == "__main__":
    simple = Simplest()
    simple.mainloop()
    rospy.spin()
