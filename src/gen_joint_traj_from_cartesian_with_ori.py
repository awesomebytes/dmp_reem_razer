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
from dmp.msg import DMPData, DMPPoint, DMPTraj
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

DEBUG_MODE = True

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
        
        self.joint_list = ['torso_1_joint', 'torso_2_joint',
                           'head_1_joint', 'head_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint', 
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.right_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        
        
#         # Store answer so we ask only one time
#         self.joint_state = JointState()
#         self.joint_state.name = gksi_answer.kinematic_solver_info.joint_names
#         self.joint_state.position = [0.0] * len(gksi_answer.kinematic_solver_info.joint_names)
#         self.ik_link_name = gksi_answer.kinematic_solver_info.link_names[0]
        self.r_commander = RobotCommander()
        self.initial_robot_state = self.r_commander.get_current_state()

        if DEBUG_MODE:
            self.pub_ok_markers = rospy.Publisher('ik_ok_marker_list', MarkerArray, latch=True)
            self.ok_markers = MarkerArray()
        
            self.pub_fail_markers = rospy.Publisher('ik_fail_marker_list', MarkerArray, latch=True)
            self.fail_markers = MarkerArray()
            self.markers_id = 5    
        
        
        
    def getIkPose(self, pose, group="right_arm", previous_state=None):
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
        if DEBUG_MODE:
            timeStart = rospy.Time.now()
        ik_answer = self.ik_serv.call(rqst)
        
        if DEBUG_MODE:
            durationCall= rospy.Time.now() - timeStart
            rospy.loginfo("Call took: " + str(durationCall.to_sec()) + "s")
        
        return ik_answer   
        
    def computeJointTrajFromCartesian(self, points, arm="right_arm_torso"):
        #fjt_goal = FollowJointTrajectoryGoal()
        poselist = []
        for point in points:
            qt = quaternion_from_euler(point.positions[3], point.positions[4], point.positions[5])
            pose = Pose(Point(point.positions[0], point.positions[1], point.positions[2]),
                        Quaternion(*qt.tolist()))
            poselist.append(pose)
        fjt_goal = self.computeIKsPose(poselist, arm)
        
        return fjt_goal
        
        
    def computeIKsPose(self, poselist, arm="right_arm"):
        rospy.loginfo("Computing " + str(len(poselist)) + " IKs" )
        fjt_goal = FollowJointTrajectoryGoal()

        if arm == 'right_arm_torso':
            fjt_goal.trajectory.joint_names = self.right_arm_torso
        elif arm == 'left_arm_torso':
            fjt_goal.trajectory.joint_names = self.left_arm_torso
        elif arm == 'right_arm':
            fjt_goal.trajectory.joint_names = self.right_arm
        elif arm == 'left_arm':
            fjt_goal.trajectory.joint_names = self.left_arm
        # TODO: add other options
        
        ik_answer = None
        for pose in poselist:
            if ik_answer != None:
                ik_answer = self.getIkPose(pose,"right_arm", previous_state=ik_answer.solution)
            else:
                ik_answer = self.getIkPose(pose)
            if DEBUG_MODE:
                rospy.loginfo("Got error_code: " + str(ik_answer.error_code.val) + " which means: " + moveit_error_dict[ik_answer.error_code.val])
            if moveit_error_dict[ik_answer.error_code.val] == 'SUCCESS':
                if DEBUG_MODE:
                    arrow = self.createArrowMarker(pose, ColorRGBA(0,1,0,1))
                    self.ok_markers.markers.append(arrow)
                jtp = JointTrajectoryPoint()
                #ik_answer = GetConstraintAwarePositionIKResponse()
                # sort positions and add only the ones of the joints we are interested in
                positions = self.sortOutJointList(fjt_goal.trajectory.joint_names, ik_answer.solution.joint_state)
                jtp.positions = positions
                # TODO: add velocities | WILL BE DONE OUTSIDE
                # TODO: add acc? | DUNNO
                fjt_goal.trajectory.points.append(jtp)
                if DEBUG_MODE:
                    self.pub_ok_markers.publish(self.ok_markers)
                
            else:
                if DEBUG_MODE:
                    arrow = self.createArrowMarker(pose, ColorRGBA(1,0,0,1))
                    self.fail_markers.markers.append(arrow)
                    self.pub_fail_markers.publish(self.fail_markers)
                
        return fjt_goal
           
    def sortOutJointList(self, joint_name_list, joint_state):
        """ Get only the joints we are interested in and it's values and return it in
        joint_state.name and joint_state.points format"""
        if DEBUG_MODE:
            rospy.loginfo("Sorting jointlist...")
        list_to_iterate = joint_name_list      
        curr_j_s = joint_state
        ids_list = []
        position_list = []
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            position_list.append(curr_j_s.position[idx_in_message])
        return position_list

    def adaptTimesAndVelocitiesOfMsg(self, trajectory, plan, desired_final_time):
        """Adapt the times and velocities of the message for the controller
        from the times computed in the DMP and velocities 0.0, controller will take care of it"""
        rospy.loginfo("Adapting times and velocities...")
        traj = trajectory #FollowJointTrajectoryGoal()
        p = plan #GetDMPPlanResponse()
        # we should have the same number of points on each, NOPE, as we can have points that IK fails
#         if len(traj.trajectory.points) != len(p.plan.points):
#             rospy.logerr("Oops, something went wrong, different number of points")
#             rospy.logerr("generated trajectory: " + str(len(traj.trajectory.points)) + " plan: " + str(len(p.plan.points)))
#             exit(0)
        
        point = JointTrajectoryPoint()
        counter = 0
        for point in traj.trajectory.points:
            #rospy.loginfo("Step " + str(counter) + " / " + str(len(traj.trajectory.points)))
            counter += 1
            point.velocities.extend(len(point.positions) * [0.0]) # adding 0.0 as speeds
            point.time_from_start = rospy.Duration( counter * (desired_final_time / len(traj.trajectory.points)) ) 
        return traj
                
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
    plan = pickle.load(open("plan_with_ori.p", "rb"))
    print plan
    rospy.init_node("calc_traj")
    t = trajectoryConstructor()
    trajectory_goal = t.computeJointTrajFromCartesian(plan.plan.points, "right_arm")
    # compute speeds and times... which is just to divide by the total time, or something like that
    t.adaptTimesAndVelocitiesOfMsg(trajectory_goal, plan, 7.0)
    #print trajectory_goal
    #t.publish_markers()
    rospy.loginfo("Times and vels set, sending to controller!")
    from controller_sender import jointControllerSender
    j = jointControllerSender()
    j.sendGoal(trajectory_goal, 'right_arm') 
    
    