#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 30 21:15:20 2013

@author: sam
"""

import sys
import actionlib

from joint_state_grabber import jointStateGrabber

from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class jointControllerSender():

    def __init__(self):
        self.right_arm_torso_as = actionlib.SimpleActionClient('/right_arm_torso_controller/follow_joint_trajectory', FollowJointTrajectory) 
        self.left_arm_as = actionlib.SimpleActionClient('/left_arm_controller/follow_joint_trajectory', FollowJointTrajectory)
        rospy.loginfo("Waiting for right_arm_torso_controller...")
        self.right_arm_torso_as.wait_for_server()
        rospy.loginfo("Waiting for left_arm_controller...")
        self.left_arm_as.wait_for_server()
        rospy.loginfo("Ready to send stuff!")
        
    def sendGoal(self, arm='right_arm_torso', goal):
        return "everything went well"
        
        
if __name__ == '__main__':
    rospy.init_node('controller_sender')
#    if len(sys.argv) < 2:
#        print "Error, we need an arg!"
#        rospy.loginfo("No args given, closing...")
#        exit()

    grabber = jointStateGrabber()
    #fjt_goal = grabber.
    node = jointControllerSender()

    node.sendGoal() # defaults to right arm torso
    
    
    #rospy.spin()
        