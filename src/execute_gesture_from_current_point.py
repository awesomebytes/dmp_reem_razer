#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tuesday December 31 18:01:53 2013

@author: sampfeiffer
"""
import sys
import actionlib
import rospy
import rosbag
from datetime import datetime
from razer_hydra.msg import Hydra
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from dmp.srv import *
from dmp.msg import *
from moveit_msgs.msg import MoveGroupGoal, MoveGroupResult, MoveGroupAction, Constraints, MoveItErrorCodes, JointConstraint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import subprocess, yaml
import math

from controller_sender import jointControllerSender
from gen_joint_traj_from_cartesian_with_ori import trajectoryConstructor
from dmp_reem_with_orientation import makePlanRequest

HYDRA_DATA_TOPIC = '/hydra_calib'
HAND_GRASP_CONTROLLER_RIGHT_AS = '/right_hand_controller/grasp_posture_controller'
HAND_GRASP_CONTROLLER_LEFT_AS = '/left_hand_controller/grasp_posture_controller'

RIGHT_HAND_POSESTAMPED_TOPIC = '/teleop_right_hand_pose'
LEFT_HAND_POSESTAMPED_TOPIC = '/teleop_left_hand_pose'
RIGHT_HAND_REFERENCE_POSESTAMPED_TOPIC = '/teleop_right_hand_pose_reference'
LEFT_HAND_REFERENCE_POSESTAMPED_TOPIC = '/teleop_left_hand_pose_reference'

PATH3D_TOPIC = '/path3d'
POSEARRAY_3D_TOPIC = '/posearray3d'

RIGHT_HAND_INITIAL_POINT = Point(x=0.6, y=-0.2, z=1.1)
#RIGHT_HAND_INITIAL_POINT = Point(x=0.6, y=-0.2, z=0.3) # For REEM-c
LEFT_HAND_INITIAL_POINT = Point(x=0.6, y=0.2, z=1.1)


# rosrun tf static_transform_publisher 0.0 -1.0 1.0 0 0 0 /base_link /hydra_base 100

class RazerControlGesture():

    def __init__(self):
        self.pub_right_hand_pose = rospy.Publisher(RIGHT_HAND_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_right_hand_pose_reference = rospy.Publisher(RIGHT_HAND_REFERENCE_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_left_hand_pose = rospy.Publisher(LEFT_HAND_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.pub_left_hand_pose_reference = rospy.Publisher(LEFT_HAND_REFERENCE_POSESTAMPED_TOPIC, PoseStamped, latch=True)
        self.hydra_data_subs = rospy.Subscriber(HYDRA_DATA_TOPIC, Hydra, self.hydraDataCallback)

        self.last_hydra_message = None
        self.current_left_pose = None
        self.current_right_pose = None
        # Store the initial and final positions of the gesture
        self.gesture_x0 = None
        self.gesture_goal = None
        self.gesture_difference = []
        self.resp_from_makeLFDRequest = None
        self.info_bag = None
        
        self.traj_constructor = trajectoryConstructor()
        self.joint_controller_sender = jointControllerSender()
        
        rospy.loginfo("Connecting to move_group AS")
        self.moveit_ac = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        self.moveit_ac.wait_for_server()
        rospy.loginfo("Succesfully connected.")


    def loadGestureFromBag(self, bagname):
        """Load gesture from the bag name given """
        # get bag info
        self.info_bag = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagname],
                                                    stdout=subprocess.PIPE).communicate()[0])
        bases_rel_to_time = math.ceil(self.info_bag['duration'] * 20) # empirically for every second 20 bases it's ok
        
        
        # Create a DMP from a 3-D trajectory with orientation
        dims = 6
        dt = 1.0
        K = 100
        D = 2.0 * np.sqrt(K)
        num_bases = bases_rel_to_time
    
        # Fill up traj with real trajectory points  
        traj = []  
        bag = rosbag.Bag(bagname)
        first_point = True
        for topic, msg, t in bag.read_messages(topics=['/teleop_right_hand_pose']):
            p = msg # PoseStamped()
            roll, pitch, yaw = euler_from_quaternion([p.pose.orientation.x,
                                   p.pose.orientation.y,
                                   p.pose.orientation.z,
                                   p.pose.orientation.w])
            traj.append([p.pose.position.x, p.pose.position.y, p.pose.position.z, roll, pitch, yaw])
            if first_point:
                # Store first point
                self.gesture_x0 = [p.pose.position.x, p.pose.position.y, p.pose.position.z, roll, pitch, yaw]
                first_point = False
        bag.close()
        # Store last point
        self.gesture_goal = [p.pose.position.x, p.pose.position.y, p.pose.position.z, roll, pitch, yaw]
        
        # Calculate the difference between initial and final point
        for val1, val2 in zip(self.gesture_x0, self.gesture_goal):                     
            self.gesture_difference.append(val2-val1)
        
        print str(len(traj)) + " points in example traj."
        resp = self.makeLFDRequest(dims, traj, dt, K, D, num_bases)
        #Set it as the active DMP
        self.makeSetActiveRequest(resp.dmp_list)
        self.resp_from_makeLFDRequest = resp
        


        
        

    def makeLFDRequest(self, dims, traj, dt, K_gain,
                       D_gain, num_bases):
        """Learn a DMP from demonstration data """
        demotraj = DMPTraj()
    
        for i in range(len(traj)):
            pt = DMPPoint();
            pt.positions = traj[i]
            demotraj.points.append(pt)
            demotraj.times.append(dt*i)
    
        k_gains = [K_gain]*dims
        d_gains = [D_gain]*dims
    
        print "Starting LfD..."
        rospy.wait_for_service('learn_dmp_from_demo')
        try:
            lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
            resp = lfd(demotraj, k_gains, d_gains, num_bases)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "LfD done"
    
        return resp

    
    def makeSetActiveRequest(self, dmp_list):
        """Set a DMP as active for planning """
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    
    def makePlanRequest(self, x_0, x_dot_0, t_0, goal, goal_thresh,
                        seg_length, tau, dt, integrate_iter):
        """Generate a plan from a DMP """
        print "Starting DMP planning..."
        rospy.wait_for_service('get_dmp_plan')
        try:
            gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
            resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                       seg_length, tau, dt, integrate_iter)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        print "DMP planning done"
    
        return resp

    def getPlanForCurrentPose(self):
        """Generate a plan for the current position of the hydra controller"""
        roll, pitch, yaw = euler_from_quaternion([
                               self.current_right_pose.pose.orientation.x,
                               self.current_right_pose.pose.orientation.y,
                               self.current_right_pose.pose.orientation.z,
                               self.current_right_pose.pose.orientation.w  ])
        x_0 = [self.current_right_pose.pose.position.x, self.current_right_pose.pose.position.y, self.current_right_pose.pose.position.z,
               roll, pitch, yaw]
        #x_0 = [0.137,-0.264,1.211,0.0395796940422, 0.0202532964694, 0.165785921829]
        x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0]
        t_0 = 0
        
        # Create the goal position, which is the initial position + the difference from the final point on the
        # original trained gesture
        goal = []
        for val1, val2 in zip(x_0, self.gesture_difference):
            goal.append( val1 + val2 )

        
        #goal = [0.259,-0.252,1.289, 0.0212535586323, -0.00664429330438, 0.117483470173]
        goal_thresh = [0.1,0.1,0.1, 0.1, 0.1, 0.1]
        seg_length = -1          #Plan until convergence to goal
        tau = 2 * self.resp_from_makeLFDRequest.tau       #Desired plan should take twice as long as demo
        tau = self.resp_from_makeLFDRequest.tau -1 # HEY WE NEED TO PUT -1 SEC HERE, WHY?? BUG?
        dt = 1.0
        integrate_iter = 5       #dt is rather large, so this is > 1
        plan_resp = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                               seg_length, tau, dt, integrate_iter)
    
#         print "this is the plan"
#         print plan_resp
        return plan_resp
        


    def hydraDataCallback(self, data):
        #rospy.loginfo("Received data from " + HYDRA_DATA_TOPIC)
        self.last_hydra_message = data
        tmp_pose_right = PoseStamped()
        tmp_pose_right.header.frame_id = 'base_link'
        tmp_pose_right.header.stamp = rospy.Time.now()
        tmp_pose_right.pose.position.x = self.last_hydra_message.paddles[1].transform.translation.x
        tmp_pose_right.pose.position.y = self.last_hydra_message.paddles[1].transform.translation.y
        tmp_pose_right.pose.position.z = self.last_hydra_message.paddles[1].transform.translation.z
        tmp_pose_right.pose.position.x += RIGHT_HAND_INITIAL_POINT.x
        tmp_pose_right.pose.position.y += RIGHT_HAND_INITIAL_POINT.y
        tmp_pose_right.pose.position.z += RIGHT_HAND_INITIAL_POINT.z
        tmp_pose_right.pose.orientation = self.last_hydra_message.paddles[1].transform.rotation
        
        tmp_pose_left = PoseStamped()
        tmp_pose_left.header.frame_id = 'base_link'
        tmp_pose_left.header.stamp = rospy.Time.now()
        tmp_pose_left.pose.position.x = self.last_hydra_message.paddles[0].transform.translation.x
        tmp_pose_left.pose.position.y = self.last_hydra_message.paddles[0].transform.translation.y
        tmp_pose_left.pose.position.z = self.last_hydra_message.paddles[0].transform.translation.z
        tmp_pose_left.pose.position.x += LEFT_HAND_INITIAL_POINT.x
        tmp_pose_left.pose.position.y += LEFT_HAND_INITIAL_POINT.y
        tmp_pose_left.pose.position.z += LEFT_HAND_INITIAL_POINT.z
        
        tmp_pose_left.pose.orientation = self.last_hydra_message.paddles[0].transform.rotation
        if self.last_hydra_message.paddles[1].buttons[0] == True:
            self.pub_right_hand_pose.publish(tmp_pose_right)
        if self.last_hydra_message.paddles[0].buttons[0] == True:
            self.pub_left_hand_pose.publish(tmp_pose_left)
            
        self.pub_right_hand_pose_reference.publish(tmp_pose_right)
        self.pub_left_hand_pose_reference.publish(tmp_pose_left)
        self.current_left_pose = tmp_pose_left
        self.current_right_pose = tmp_pose_right


    def create_move_group_joints_goal(self, joint_names, joint_values, group="right_arm", plan_only=False):
        """ Creates a move_group goal based on pose.
        @arg joint_names list of strings of the joint names
        @arg joint_values list of digits with the joint values
        @arg group string representing the move_group group to use
        @arg plan_only bool to for only planning or planning and executing
        @return MoveGroupGoal with the desired contents"""
        
        header = Header()
        header.frame_id = 'base_link'
        header.stamp = rospy.Time.now()
        moveit_goal = MoveGroupGoal()
        goal_c = Constraints()
        for name, value in zip(joint_names, joint_values):
            joint_c = JointConstraint()
            joint_c.joint_name = name
            joint_c.position = value
            joint_c.tolerance_above = 0.01
            joint_c.tolerance_below = 0.01
            joint_c.weight = 1.0
            goal_c.joint_constraints.append(joint_c)
    
        moveit_goal.request.goal_constraints.append(goal_c)
        moveit_goal.request.num_planning_attempts = 5
        moveit_goal.request.allowed_planning_time = 5.0
        moveit_goal.planning_options.plan_only = plan_only
        moveit_goal.planning_options.planning_scene_diff.is_diff = True
        moveit_goal.request.group_name = group
        
        return moveit_goal



    def run(self):
        rospy.loginfo("Press LB / RB to send the current pose")
        
        while self.last_hydra_message == None:
            rospy.sleep(0.1)
            
        rospy.loginfo("Got the first data of the razer... Now we can do stuff")
            
        sleep_rate=0.05 # check at 20Hz
        left_pushed = right_pushed = False
        
        while True:   
            if self.last_hydra_message.paddles[1].buttons[0] == True and not right_pushed:
                right_pushed = True
                rospy.loginfo("Pressed RB")
                rospy.loginfo("Creating plan from current pose...")
                plan = self.getPlanForCurrentPose()
                #print plan
                rospy.loginfo("Creating trajectory from the plan...")
                trajectory_goal = self.traj_constructor.computeJointTrajFromCartesian(plan.plan.points, "right_arm")
                # compute speeds and times... which is just to divide by the total time, or something like that
                rospy.loginfo("Adapting times and velocities on the trajectory...")
                self.traj_constructor.adaptTimesAndVelocitiesOfMsg(trajectory_goal, plan, self.info_bag['duration']) # Maybe make the movement longer than the original time
                rospy.loginfo("Times and vels set, starting movement!")
                rospy.loginfo("Sending first pose to moveit so we dont fail on points of the trajectory")
#                 a = FollowJointTrajectoryGoal()
                joint_names = trajectory_goal.trajectory.joint_names
                joint_values = trajectory_goal.trajectory.points[0].positions
                group="right_arm"
                plan_only=False
                goal = self.create_move_group_joints_goal(joint_names, joint_values,group,plan_only)
                self.moveit_ac.send_goal(goal)
                self.moveit_ac.wait_for_result()
                rospy.loginfo("Sending full trajectory to controller")
                self.joint_controller_sender.sendGoal(trajectory_goal, 'right_arm') 
                
                
            elif self.last_hydra_message.paddles[1].buttons[0] == False and right_pushed:
                right_pushed = False

                rospy.loginfo("Released RB")
                
                
            rospy.sleep(sleep_rate)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Name of the recorded bag needed"
        exit(0)
    else:
        print "Will use bag with name: " + str(sys.argv[1])
        bagname = sys.argv[1]
    rospy.init_node('hydra_get_point_to_execute_gesture')
#    if len(sys.argv) < 2:
#        print "Error, we need an arg!"
#        rospy.loginfo("No args given, closing...")
#        exit()

    node = RazerControlGesture()
    node.loadGestureFromBag(bagname)
    
    node.run()
