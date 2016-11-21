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
from move_base_msgs.msg import MoveBaseGoal
import math

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

def enemy_detect_cb(ud, msg):
    if msg.yaw == 999.0 and msg.pitch == 999.0:
        global now_state
        now_state = 0
        return True
    else:
        global now_state
        now_state = 1
        return False

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

# def blood_state_cb(ud, msg):
#     if self.side == 'blue':
#         for blood in msg.redtank_blood :
#             if blood == 0:
#
#     else:








def get_new_searching_point():
        # searching_point1 = MoveBaseGoal()
        # searching_point1.target_pose.header.frame_id = 'map'
        # searching_point1.target_pose.header.stamp = rospy.Time.now()
        # searching_point1.target_pose.pose.position.x = 70.25
        # searching_point1.target_pose.pose.position.y = 7.065
        # searching_point1.target_pose.pose.orientation.w = 1.0
        #
        # searching_point2 = MoveBaseGoal()
        # searching_point2.target_pose.header.frame_id = 'map'
        # searching_point2.target_pose.header.stamp = rospy.Time.now()
        # searching_point2.target_pose.pose.position.x = 70.134
        # searching_point2.target_pose.pose.position.y = -1.716
        # searching_point2.target_pose.pose.orientation.w = 1.0
        #
        # searching_point3 = MoveBaseGoal()
        # searching_point3.target_pose.header.frame_id = 'map'
        # searching_point3.target_pose.header.stamp = rospy.Time.now()
        # searching_point3.target_pose.pose.position.x = 66.866
        # searching_point3.target_pose.pose.position.y = 0.052
        # searching_point3.target_pose.pose.orientation.w = 1.0
        #
        # searching_point4 = MoveBaseGoal()
        # searching_point4.target_pose.header.frame_id = 'map'
        # searching_point4.target_pose.header.stamp = rospy.Time.now()
        # searching_point4.target_pose.pose.position.x = 67.639
        # searching_point4.target_pose.pose.position.y = 5.962
        # searching_point4.target_pose.pose.orientation.w = 1.0

        searching_point1 = MoveBaseGoal()
        searching_point1.target_pose.header.frame_id = 'map'
        searching_point1.target_pose.header.stamp = rospy.Time.now()
        searching_point1.target_pose.pose.position.x = 14.155
        searching_point1.target_pose.pose.position.y = 28.70
        searching_point1.target_pose.pose.orientation.w = 1.0

        searching_point2 = MoveBaseGoal()
        searching_point2.target_pose.header.frame_id = 'map'
        searching_point2.target_pose.header.stamp = rospy.Time.now()
        searching_point2.target_pose.pose.position.x = 19.15
        searching_point2.target_pose.pose.position.y = 28.70
        searching_point2.target_pose.pose.orientation.w = 1.0

        searching_point3 = MoveBaseGoal()
        searching_point3.target_pose.header.frame_id = 'map'
        searching_point3.target_pose.header.stamp = rospy.Time.now()
        searching_point3.target_pose.pose.position.x = 18.90
        searching_point3.target_pose.pose.position.y = 25.0
        searching_point3.target_pose.pose.orientation.w = 1.0

        searching_point4 = MoveBaseGoal()
        searching_point4.target_pose.header.frame_id = 'map'
        searching_point4.target_pose.header.stamp = rospy.Time.now()
        searching_point4.target_pose.pose.position.x = 15.24
        searching_point4.target_pose.pose.position.y = 24.85
        searching_point4.target_pose.pose.orientation.w = 1.0

        searching_points = [searching_point1, searching_point2, searching_point3, searching_point4]
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
            nearest_point = MoveBaseGoal()
            nearest_dis = 100000.0
            for point in searching_points :
                if math.hypot(point.target_pose.pose.position.x - trans[0], point.target_pose.pose.position.y - trans[1]) < nearest_dis and math.hypot(point.target_pose.pose.position.x - trans[0], point.target_pose.pose.position.y - trans[1]) > 0.6:
                    nearest_point = point
                    nearest_dis = math.hypot(point.target_pose.pose.position.x - trans[0], point.target_pose.pose.position.y - trans[1])
            nearest_point.target_pose.header.stamp = rospy.Time.now()
            nearest_point.target_pose.header.frame_id = 'map'
            nearest_point.target_pose.pose.orientation.z = rot[2]
            nearest_point.target_pose.pose.orientation.w = rot[3]
            print nearest_point
            return nearest_point
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
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
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
        self.client.send_goal(now_point)
        while self.client.get_state() == 1 or self.client.get_state() == 0:# the state is active,  goal is running
            print "executing"
            if self.preempt_requested():
                print "now goal is being preempted!!!"
                self.service_preempt()
                return 'preempted'
        # self.client.wait_for_result()
        return 'done'


    def service_preempt(self):
        State.request_preempt(self)
        if self.goal_sent:
            print "goal canceled"
            self.client.cancel_goal()


class With_blood:
    def set_safety(self, msg):
        safety_ = safety()
        if msg == "lock" :
            safety_.safe.data = False
            self.safety_pub.publish(safety_)
        else:
            safety_.safe.data = True
            self.safety_pub.publish(safety_)

    def __init__(self):
        rospy.init_node('Simplest')
        self.safety_pub = rospy.Publisher("safety",safety)
        #define which side I am
        self.side = rospy.get_param('My_side', 'blue')

    def mainloop(self):
        global  now_state
        now_state = 0
        Searching_moutain = Concurrence(outcomes=['searching', 'engaging'],default_outcome='searching', child_termination_cb=searching_term_cb, outcome_cb=searching_out_cb)
        with Searching_moutain:
            Concurrence.add('nav_goal', Nav_next_point())
            Concurrence.add('enemy_detect', MonitorState("cmd_gimbal", gimbal, enemy_detect_cb))
            Concurrence.add('death_detect', MonitorState("blood_state", blood, blood_state_cb))
        Engaging = Concurrence(outcomes=['searching', 'engaging'],default_outcome='searching', child_termination_cb=engaging_term_cb, outcome_cb=engaging_out_cb)
        with Engaging:
            Concurrence.add('nav_goal', Nav_next_point())
            Concurrence.add('enemy_detect', MonitorState("cmd_gimbal", gimbal, enemy_detect_cb),remapping={'invalid': 'valid', 'valid': 'invalid'})
            Concurrence.add('death_detect', MonitorState("blood_state", blood, blood_state_cb))
        sm = StateMachine(outcomes=['ALL_DONE'])
        with sm:
            StateMachine.add('SEARCHING', Searching_moutain, transitions={'searching': 'SEARCHING', 'engaging': 'ENGAGING'})
            StateMachine.add('ENGAGING', Engaging, transitions={'searching': 'SEARCHING', 'engaging': 'ENGAGING'})
        sm.execute()

if __name__ == "__main__":
    simple = With_blood()
    With_blood.mainloop()
    rospy.spin()
