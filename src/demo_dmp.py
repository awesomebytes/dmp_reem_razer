#!/usr/bin/env python
import roslib;
roslib.load_manifest('dmp_reem_movements')
import rospy
import numpy as np
from dmp.srv import *
from dmp.msg import *

from pylab import *
import matplotlib.pyplot as plt

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
        plt.plot([dt*i for i in range(len(traj))], [row[dim] for row in traj], 'ko-')
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
        plt.plot(timelist, positionlist, 'ko-')
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
        plt.plot(timelist, velocitieslist, 'ko-')
        plt.ylabel('vel ' +  str(dim + 1))
        plt.xlabel('time (s)')
        grid(True)
        ylim([-ylimit,ylimit])
        numplot+=1


    savefig("traj_gen_" + str(name) + ".png")



if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 2-D trajectory
    dims = 2
    dt = 1.0
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 4
    traj = [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)


    saveGraphTraj(dims, traj, dt, "1", ylimit=10)

    print "lfdrequest resp is"
    print resp


    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [0.0,0.0]          #Plan starting at a different point than demo
    x_0 = [1.0,1.0]
    x_dot_0 = [0.0,0.0]
    t_0 = 0
    goal = [8.0,7.0]  #Plan to a different goal than demo
    goal = [6.0,8.0]
    goal_thresh = [0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = 2 * resp.tau       #Desired plan should take twice as long as demo
    tau = resp.tau -1 # HEY WE NEED TO PUT -1 SEC HERE, WHY?? BUG?
    dt = 1.0
    integrate_iter = 5       #dt is rather large, so this is > 1
    plan_resp = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    print "this is the plan"
    print plan_resp


    saveGraphTrajGenerated(dims, x_0, plan_resp.plan, "1", ylimit=10)
