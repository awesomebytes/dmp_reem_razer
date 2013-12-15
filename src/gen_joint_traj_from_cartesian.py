#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Dec 14 19:08:11 2013

@author: Sam Pfeiffer
"""

import rospy
import numpy as np
import sys
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from dmp.srv import *
from dmp.msg import *
import pickle

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIKResponse, GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes


# build a mapping from arm navigation error codes to error names
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


# IK_SERVICE_NAME = '/reem_right_arm_torso_kinematics/get_constraint_aware_ik'
# IK_SOLVER_INFO_SERVICE_NAME = '/reem_right_arm_torso_kinematics/get_ik_solver_info'
# 
# IK_SERVICE_NAME = '/reem_right_arm_kinematics/get_constraint_aware_ik'
# IK_SOLVER_INFO_SERVICE_NAME = '/reem_right_arm_kinematics/get_ik_solver_info'

IK_SERVICE_NAME = '/compute_ik'

class trajectoryConstructor():
    def __init__(self):
        rospy.loginfo("Waiting for service " + IK_SERVICE_NAME)
        rospy.wait_for_service(IK_SERVICE_NAME)
        self.ik_serv = rospy.ServiceProxy(IK_SERVICE_NAME, GetPositionIK)
        
#         # Store answer so we ask only one time
#         self.joint_state = JointState()
#         self.joint_state.name = gksi_answer.kinematic_solver_info.joint_names
#         self.joint_state.position = [0.0] * len(gksi_answer.kinematic_solver_info.joint_names)
#         self.ik_link_name = gksi_answer.kinematic_solver_info.link_names[0]
        self.r_commander = RobotCommander()
        self.initial_robot_state = self.r_commander.get_current_state()

        self.pub_ok_markers = rospy.Publisher('ik_ok_marker_list', MarkerArray, latch=True)
        self.ok_markers = MarkerArray()
        
        self.pub_fail_markers = rospy.Publisher('ik_fail_marker_list', MarkerArray, latch=True)
        self.fail_markers = MarkerArray()
        self.markers_id = 5    
        
        
        
    def getIkPose(self, pose, group="right_arm_torso", previous_state=None):
        """Get IK of the pose specified, for the group specified, optionally using
        the robot_state of previous_state (if not, current robot state will be requested) """
        # point point to test if there is ik
        # returns the answer of the service
        rqst = GetPositionIKRequest()
        rqst.ik_request.avoid_collisions = True
        rqst.ik_request.group_name = group
        rqst.ik_request.pose_stamped.header = Header(stamp=rospy.Time.now())
        rqst.ik_request.pose_stamped.header.frame_id = 'base_link'


        # Set point to check IK for
        rqst.ik_request.pose_stamped.pose.position = pose.position
        rqst.ik_request.pose_stamped.pose.orientation = pose.orientation
        
        if previous_state == None:
            cs = self.r_commander.get_current_state()
            rqst.ik_request.robot_state = cs
        else:
            rqst.ik_request.robot_state = previous_state

        ik_answer = GetPositionIKResponse()
        timeStart = rospy.Time.now()
        ik_answer = self.ik_serv.call(rqst)
        durationCall= rospy.Time.now() - timeStart
        rospy.loginfo("Call took: " + str(durationCall.to_sec()) + "s")
        
        return ik_answer   
        
    def computeJointTrajFromCartesian(self, points):
        #fjt_goal = FollowJointTrajectoryGoal()
        # orientation problem is still pending, so for now same quaternion for every point
        generic_ori = Quaternion(w=1.0)
        poselist = []
        for point in points:
            pose = Pose(Point(point.positions[0], point.positions[1], point.positions[2]),
                        generic_ori)
            poselist.append(pose)
        fjt_goal = self.computeIKsPose(poselist)
        # add velocities?
        
        
        return fjt_goal
        
        
    def computeIKsPose(self, poselist):
        rospy.loginfo("Computing " + str(len(poselist)) + " IKs" )
        fjt_goal = FollowJointTrajectoryGoal()
        #names_right_arm_torso = 
        fjt_goal.trajectory.joint_names = self.initial_robot_state.joint_state.name
        
        ik_answer = None
        for pose in poselist:
            if ik_answer != None:
                ik_answer = self.getIkPose(pose, previous_state=ik_answer.solution)
            else:
                ik_answer = self.getIkPose(pose)
            rospy.loginfo("Got error_code: " + str(ik_answer.error_code.val) + " which means: " + moveit_error_dict[ik_answer.error_code.val])
            if moveit_error_dict[ik_answer.error_code.val] == 'SUCCESS':
                arrow = self.createArrowMarker(pose, ColorRGBA(0,1,0,1))
                self.ok_markers.markers.append(arrow)
                jtp = JointTrajectoryPoint()
                #ik_answer = GetConstraintAwarePositionIKResponse()
                jtp.positions = ik_answer.solution.joint_state.position
                # TODO: add velocities
                # TODO: add acc?
                fjt_goal.trajectory.points.append(jtp)
                self.pub_ok_markers.publish(self.ok_markers)
                
            else:
                arrow = self.createArrowMarker(pose, ColorRGBA(1,0,0,1))
                self.fail_markers.markers.append(arrow)
                self.pub_fail_markers.publish(self.fail_markers)
                
        return fjt_goal
                
    def publish_markers(self):
        while True:
            self.pub_ok_markers.publish(self.ok_markers)
            self.pub_fail_markers.publish(self.fail_markers)
            rospy.sleep(0.1)
        
    def createArrowMarker(self, pose, color):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.type = marker.ARROW
        marker.action = marker.ADD
        general_scale = 0.01
        marker.scale.x = general_scale
        marker.scale.y = general_scale / 3.0
        marker.scale.z = general_scale / 10.0
        marker.color = color
        marker.pose.orientation = pose.orientation
        marker.pose.position = pose.position
        marker.id = self.markers_id
        self.markers_id += 1
        return marker

if __name__ == '__main__':
    plan = pickle.load(open("plan_.p", "rb"))
    print plan
    rospy.init_node("calc_traj")
    t = trajectoryConstructor()
    trajectory_goal = t.computeJointTrajFromCartesian(plan.plan.points)
    print trajectory_goal
    t.publish_markers()
    
    