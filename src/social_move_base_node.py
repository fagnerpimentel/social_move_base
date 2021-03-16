#!/usr/bin/env python
import copy
import rospy
import numpy

from actionlib import SimpleActionServer, SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from social_move_base.msg import SocialMoveBaseAction, SocialMoveBaseGoal
from social_move_base.msg import SocialMoveBaseFeedback, SocialMoveBaseResult
from social_msgs.msg import Locals, Local
from social_msgs.msg import People, Person
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from std_msgs.msg import Header

from util import *

class SocialMoveBaseNode:

    def __init__(self):

        # general variables
        self.locals = {}
        self.people = {}
        self.robot_pose = Pose()
        self.costmap_global = OccupancyGrid()
        self.target_last_move_time = rospy.get_time()

        # actual action variables
        self.target_name = ''
        self.target_type = ''
        # self.navigation_type = ''
        # self.approach_dist = ''
        # self.approach_ang = ''
        # self.last_target = None
        self.actual_target = None
        self.approach_target = None
        self.safe_target = None


        self.global_planner = rospy.get_param('~global_planner', '')

        self.sub_locals = init_subscriber('/locals', Locals, self.__callback_sub_locals__)
        self.sub_people = init_subscriber('/people', People, self.__callback_sub_people__)

        self.sub_robot = init_subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__callback_sub_robot__)
        # self.sub_costmap_local = init_subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.__callback_sub_costmap_local__)
        self.sub_costmap_global = init_subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.__callback_sub_costmap_global__)
        rospy.loginfo('Subscribers ready.')

        self.scl_pathplan = init_service_client('/move_base/{}/make_plan'.format(self.global_planner.split("/", 1)[1]),GetPlan)
        rospy.loginfo('Services client ready.')

        self.ase_social_navigation = init_action_server('/social_navigation', SocialMoveBaseAction, self.__callback_ase_social_navigation__)
        rospy.loginfo('Actions server ready.')

        self.acl_move_base = init_action_client('/move_base', MoveBaseAction)
        rospy.loginfo('Actions client ready.')

        rospy.loginfo('Social navigation ready!')

    def __callback_sub_locals__(self, data):
        # self.locals = data.locals
        # self.locals = {}
        for local in data.locals:
            self.locals[local.name] = local

        # if(self.target_type == 'LOCAL' and self.target_name in self.locals.keys()):
        #     self.target_last_move_time = rospy.get_time()

    def __callback_sub_people__(self, data):
        # self.people = data.people
        # self.people = {}
        for person in data.people:
            self.people[person.name] = person

        # if(self.target_type == 'PERSON' and self.target_name in self.people.keys()):
        #     self.target_last_move_time = rospy.get_time()

    def __callback_sub_robot__(self, data):
        self.robot_pose = data.pose.pose
    # def __callback_sub_costmap_local__(self, data):
    #     self.costmap_local = data
    def __callback_sub_costmap_global__(self, data):
        self.costmap_global = data
    def __callback_sub_costmap_global__(self, data):
        self.costmap_global = data

    def set_new_target(self):

        if self.target_type == 'LOCAL':
            local = self.locals[self.target_name]
            self.actual_target = copy.deepcopy(local.pose)
            self.approach_target = copy.deepcopy(local.pose)
            self.safe_target = copy.deepcopy(local.pose)

        if self.target_type == 'PERSON':
            person = self.people[self.target_name]
            self.actual_target = copy.deepcopy(person.pose)
            self.approach_target = copy.deepcopy(person.pose_approach)
            self.safe_target = copy.deepcopy(person.pose_approach)


    def __callback_ase_social_navigation__(self, goal):
        _result = SocialMoveBaseResult()

        if(goal.target_name != ''):
            self.target_name = goal.target_name
        else:
            _result.result = 'Invalid destination.'
            self.ase_social_navigation.set_aborted(_result)
            rospy.loginfo('ABORTED')
            return

        if self.target_name in self.locals.keys():
            self.target_type = 'LOCAL'
        elif self.target_name in self.people.keys():
            self.target_type = 'PERSON'
        else:
            _result.result = 'Destination not founded.'
            self.ase_social_navigation.set_aborted(_result)
            rospy.loginfo('ABORTED')
            return

        rospy.loginfo('Target name: {}'.format(self.target_name))
        rospy.loginfo('Target type: {}'.format(self.target_type))

        diff = 0
        while diff < 5:
            self.set_new_target()
            target = self.get_free_pose(self.robot_pose, self.approach_target)
            self.send_goal(target)
            self.acl_move_base.wait_for_result()
            now = rospy.get_time()
            diff = now - self.target_last_move_time
            print (diff)

        self.ase_social_navigation.set_succeeded(_result)
        rospy.loginfo('SUCCEEDED')


    # def get_target(self, target_name):
    #
    #     success = False
    #     original_pose = Pose()
    #     final_pose = Pose()
    #
    #     locals = [local for local in self.locals if local.name == target_name]
    #     people = [person for person in self.people if person.name == target_name]
    #
    #     if len(locals) > 0:
    #         return [True, copy.deepcopy(locals[0].pose), copy.deepcopy(locals[0].pose), 'local']
    #     elif len(people) > 0:
    #         return [True, copy.deepcopy(people[0].pose), copy.deepcopy(people[0].pose_approach), 'person']
    #     else:
    #         success = [False,Pose(),Pose(),'']

    # def find_approach_pose(self, target):
    #     original_target = copy.deepcopy(target)
    #     new_target = copy.deepcopy(target)
    #
    #     [yaw,_,_] = quaternion_to_euler(
    #         target.orientation.x,target.orientation.y,
    #         target.orientation.z,target.orientation.w)
    #
    #     d = 1.2
    #     dx = d * numpy.cos(yaw)
    #     dy = d * numpy.sin(yaw)
    #     new_target.position.x += dx
    #     new_target.position.y += dy
    #     new_target.position.z = 0;
    #
    #     alfa = numpy.arctan2(
    #         new_target.position.y - original_target.position.y,
    #         new_target.position.x - original_target.position.x)
    #     q = euler_to_quaternion(0,0,alfa)
    #     new_target.orientation.x = q[0]
    #     new_target.orientation.y = q[1]
    #     new_target.orientation.z = q[2]
    #     new_target.orientation.w = q[3]
    #
    #     return new_target

    def send_goal(self, target):
        self.safe_target = copy.deepcopy(target)
        self.acl_move_base.cancel_goal()
        goal = MoveBaseGoal()
        goal.target_pose.header = Header(0,rospy.Time.now(),"map")
        goal.target_pose.pose = target
        self.acl_move_base.send_goal(goal,
            active_cb=self.__callback_acl_move_base_active__,
            feedback_cb=self.__callback_acl_move_base_feedback__,
            done_cb=self.__callback_acl_move_base_done__)

    def __callback_acl_move_base_active__(self):
        rospy.loginfo('Action server "navigation": Processing the goal.')
    def __callback_acl_move_base_done__(self, state, result):
        rospy.loginfo('Action server "navigation": Done (State: {}, result: {}).'.format(state,result))
    def __callback_acl_move_base_feedback__(self, feedback):
        # rospy.loginfo('Action server "navigation": Feedback - {}.'.format(feedback))

        if(self.target_type == 'LOCAL'):
            p1 = self.locals[self.target_name].pose.position
        if(self.target_type == 'PERSON'):
            p1 = self.people[self.target_name].pose.position
        p2 = self.actual_target.position
        dist = numpy.sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2))

        if(dist > 0.1):
            rospy.loginfo('Target has moved.')
            self.target_last_move_time = rospy.get_time()

            self.set_new_target()
            target = self.get_free_pose(self.robot_pose, self.approach_target)
            self.send_goal(target)

        if(not self.costmap_is_free_at_position(self.safe_target.position)):
            rospy.loginfo('Target at danger area.')
            target = self.get_free_pose(self.robot_pose, self.approach_target)
            self.send_goal(target)


    def get_free_pose(self, start, goal):

        # goal_aproach = copy.deepcopy(goal)
        # if(type == 'person'):
        #     goal_aproach = self.find_approach_pose(goal)

        plan = Path()
        srv = GetPlan()
        ps_start = PoseStamped(Header(0,rospy.Time.now(),"map"), start)
        ps_goal = PoseStamped(Header(0,rospy.Time.now(),"map"), goal)

        rospy.loginfo('Finding a path.')
        while(len(plan.poses) == 0):
            plan = self.scl_pathplan(ps_start,ps_goal,0.1).plan

        for pose in reversed(plan.poses):
            if(self.costmap_is_free_at_position(pose.pose.position)):
                p1 = pose.pose.position
                p2 = self.actual_target.position
                dist = numpy.sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2))
                if(dist != 0):
                    alfa = numpy.arctan2(p2.y-p1.y,p2.x-p1.x)
                    q = euler_to_quaternion(0,0,alfa)
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                return pose.pose
        return self.robot_pose


    def costmap_is_free_at_position(self, position):
            x = position.x-self.costmap_global.info.origin.position.x
            y = position.y-self.costmap_global.info.origin.position.y
            pixel_x = round(x/self.costmap_global.info.resolution)
            pixel_y = round(y/self.costmap_global.info.resolution)
            i = int(pixel_y * self.costmap_global.info.width + pixel_x)
            if self.costmap_global.data[i] == 0:
                return True
            else:
                return False

if __name__ == '__main__':
    try:
        rospy.init_node('social_move_base_node')
        SocialMoveBaseNode()
    except KeyboardInterrupt:
        pass
