#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sat Dec  7 17:11:57 2013

@author: Sam Pfeiffer
"""

import rospy
import numpy as np
import sys
from dmp.srv import *
from dmp.msg import *
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
import rosbag
from pylab import *
import matplotlib.pyplot as plt
import pickle

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain,
                   D_gain, num_bases):
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

    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"

    return resp;


def saveGraphTraj(dimensions, traj, dt, name="default", ylimit=100):
    for dim in range(dimensions):
        plt.subplot(dimensions, 1, dim+1)
        # x is times, or steps, y is the dimension data
        plt.plot([dt*i for i in range(len(traj))], [row[dim] for row in traj], 'k')
        plt.ylabel('dim ' +  str(dim + 1))
        plt.xlabel('time (s)')
        grid(True)

        ylim([-ylimit,ylimit])

    savefig("traj_" + str(name) + ".png")


def saveGraphTrajGenerated(dimensions, x_0, plan, name="default", ylimit=100):
    plt.close()
    numplot = 1

    # seems like the plan doesnt include initial time, BUG?
    timelist = [0.0]
    timelist.extend(plan.times)

    for dim in range(dimensions):
        # First positions
        # subplot ( nrows ncols numplot )
        plt.subplot(dimensions, 2, numplot)
        # x is times, or steps, y is the dimension data
        # seems that the plan generated doesnt include the first point for some reason probably A BUG!!
        positionlist = [x_0[dim]]
        positionlist.extend([point.positions[dim] for point in plan.points])
        plt.plot(timelist, positionlist, 'k')
        plt.ylabel('pos ' +  str(dim + 1))
        plt.xlabel('time (s)')
        grid(True)
        ylim([-ylimit,ylimit])
        numplot+=1

        # Then velocities
        plt.subplot(dimensions, 2, numplot)
        # x is times, or steps, y is the dimension data
        velocitieslist = [0.0]
        velocitieslist.extend([point.velocities[dim] for point in plan.points])
        plt.plot(timelist, velocitieslist, 'k')
        plt.ylabel('vel ' +  str(dim + 1))
        plt.xlabel('time (s)')
        grid(True)
        ylim([-ylimit,ylimit])
        numplot+=1


    savefig("traj_gen_" + str(name) + ".png")



if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Name of the recorded bag needed"
        exit(0)
    else:
        print "Will use bag with name: " + str(sys.argv[1])
        bagname = sys.argv[1]
    
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 3-D trajectory
    dims = 3
    dt = 1.0
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 40

    # Fill up traj with real trajectory points  
    traj = []  
    bag = rosbag.Bag(bagname)
    for topic, msg, t in bag.read_messages(topics=['/teleop_right_hand_pose']):
        p = msg # PoseStamped()
        traj.append([p.pose.position.x, p.pose.position.y, p.pose.position.z])
    bag.close()
    #print traj
    print str(len(traj)) + " points in example traj."
    
    #traj = [[1.0,1.0,1.0],[2.0,2.0,2.0],[3.0,4.0,5.0],[6.0,8.0,10.0]]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)


    saveGraphTraj(dims, traj, dt, "1", ylimit=2)

    print "lfdrequest resp is"
    print resp


    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [0.137,-0.264,1.211] # Plan same than demo
    x_dot_0 = [0.0,0.0,0.0]
    t_0 = 0
    goal = [0.259,-0.252,1.289]  #Plan same than demo
    goal_thresh = [0.1,0.1,0.1]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    tau = resp.tau -1 # HEY WE NEED TO PUT -1 SEC HERE, WHY?? BUG?
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1
    plan_resp = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    print "this is the plan"
    print plan_resp
    
    path3d = Path()
    path3d.header.frame_id = 'base_link'
    path3d.header.stamp = rospy.Time.now()
    for point in plan_resp.plan.points:
        path3d.poses.append(PoseStamped(header=path3d.header,
                                pose=Pose(Point(point.positions[0], point.positions[1], point.positions[2]), 
                                Quaternion(w=1.0))
                                )
                                )
    pathpub = rospy.Publisher("/generated_path3d", Path, latch=True)
    pathpub.publish(path3d)


    saveGraphTrajGenerated(dims, x_0, plan_resp.plan, "1", ylimit=2)

    pickle.dump(plan_resp, open("plan_.p", "wb"))
    
    rospy.spin()
