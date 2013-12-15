#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 24 17:39:55 2013

@author: sampfeiffer
"""
import sys


from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class jointStateGrabber():

    def __init__(self):
        # my class variables
        self.current_joint_states = None
        self.joint_list = ['torso_1_joint', 'torso_2_joint',
                           'head_1_joint', 'head_2_joint',
                           'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint', 
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_left_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.left_arm = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                           'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
                           'arm_left_7_joint']
        self.right_arm = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_left_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.right_arm_torso = ['torso_1_joint', 'torso_2_joint',
                           'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                           'arm_right_4_joint', 'arm_left_5_joint', 'arm_right_6_joint',
                           'arm_right_7_joint']
        self.ids_list = []

        self.subs = rospy.Subscriber('/joint_states', JointState, self.getJointStates)
        
        # getting first message to correctly find joints
        while self.current_joint_states == None:
            rospy.sleep(0.1)
        rospy.loginfo("Node initialized. Ready to grab joint states")
        

    def getJointStates(self, data):
        #rospy.loginfo("Received from topic data!")
        self.current_joint_states = data
        
    def createGoalFromCurrentJointStateForArm(self, group='right_arm_torso'):
        """ Get the joints for the specified group and return it """
        list_to_iterate = getattr(self, group)        
        curr_j_s = self.current_joint_states
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug("Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))
    
        # TODO: put this in a useful ROS message
        fjtg = FollowJointTrajectoryGoal()
        fjtg.trajectory.joint_names.extend(list_to_iterate)
        jtp = JointTrajectoryPoint(positions=msg_list,time_from_start=0) # fast as hell
        fjtg.trajectory.points.append(jtp)
        
        rospy.loginfo("follow joint trajectory goal:\n" + str(fjtg))
        
        return fjtg
        
    

if __name__ == '__main__':
    rospy.init_node('joint_state_grabber')
#    if len(sys.argv) < 2:
#        print "Error, we need an arg!"
#        rospy.loginfo("No args given, closing...")
#        exit()

    node = jointStateGrabber()

    node.createGoalFromCurrentJointStateForArm() # defaults to right arm torso
    
    
    #rospy.spin()