#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 30 21:15:20 2013

@author: sam
"""

import sys
import actionlib
import rospy

from joint_state_grabber import jointStateGrabber

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint

class jointControllerSender():

    def __init__(self):
        self.right_arm_as = actionlib.SimpleActionClient('/right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction) 
        self.left_arm_as = actionlib.SimpleActionClient('/left_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.torso_as = actionlib.SimpleActionClient('/torso_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for right_arm_controller...")
        self.right_arm_as.wait_for_server()
        rospy.loginfo("Waiting for left_arm_controller...")
        self.left_arm_as.wait_for_server()
        rospy.loginfo("Waiting for torso_controller...")
        self.torso_as.wait_for_server()
        rospy.loginfo("Ready to send stuff!")
        
    def sendGoal(self, goal, arm):
        rospy.logwarn("Goal is: \n" + str(goal))
        if arm == 'right_arm':
            rospy.loginfo("Sending goal to right_arm...")
            self.right_arm_as.send_goal(goal)
            rospy.loginfo("Waiting for result...")
            self.right_arm_as.wait_for_result()
            rospy.loginfo("Done.")
        elif arm == 'left_arm':
            rospy.loginfo("Sending goal to left_arm...")
            self.left_arm_as.send_goal(goal)
            rospy.loginfo("Waiting for result...")
            self.left_arm_as.wait_for_result()
            rospy.loginfo("Done.")
        
        
if __name__ == '__main__':
    rospy.init_node('controller_sender')
#    if len(sys.argv) < 2:
#        print "Error, we need an arg!"
#        rospy.loginfo("No args given, closing...")
#        exit()

    #grabber = jointStateGrabber()
    #fjt_goal = grabber.
    node = jointControllerSender()
    testgoal = FollowJointTrajectoryGoal()
    node.sendGoal(testgoal, "right_arm") # defaults to right arm torso
    
    
    #rospy.spin()
        