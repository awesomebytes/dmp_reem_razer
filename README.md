dmp_reem_razer
===========

1) Capturing position/orientation (orientation pending to be added on calculations) from a razer hydra to capture gestures.

2) Computing dmp in cartersian [x, y, z] and orientation [roll, pitch, yaw] space of the end effector.

3) Generate trajectories using the active dmp for different starting and ending positions.

4) Sending the trajectories to the controllers (calculating collision aware IK's thanks to MoveIt!)


Using the ros package of dmp tests ( http://wiki.ros.org/dmp ) but updated for hydro: https://github.com/makokal/dmp
Also using razer_hydra package updated from: https://github.com/skohlbr/razer_hydra
(Current one in debs sometimes gives:
terminate called after throwing an instance of 'std::runtime_error'
  what():  Time is out of dual 32-bit range
 And stops working)

Also plotting with matplotlib the trajectories generated.

Video of it working available here:
http://www.youtube.com/watch?v=AKAeQqSU-dY


===========
How to use:

1) Launch a reem simulation (roslaunch reem_gazebo reem_empty_world.launch)
2) Launch moveit for reem with controllers (roslaunch reem_moveit_config moveit_planning_execution.launch)
3) Launch a static transform (yeah, this is something I must solve) for odom_combined which doesn't exist in this simulation: rosrun tf static_transform_publisher 0 0 0 0 0 0 odom_combined odom 100
4) Launch dmp server (roslaunch dmp dmp.launch)

To capture with Razer:
1) Launch razer hydra driver with hydra connected (roslaunch razer_hydra hydra.launch)
2) Run hydra_grab_points.py
3) Press the RB button of the controller to start recording a gesture, when you release it, the gesture ends. This will create a rosbag with the current timestamp with the Pose of the controller. (Same with LB).

To load a captured gesture and generate a new one:
1) Run dmp_reem_with_orientation.py [name_of_bag]
2) This will load the captured gesture and then create a new plan (line of code 179+) from new starting and ending points.
3) It will dump the generated plan with pickle in a .p extension file (change it's name or it will overwrite the last one!).

To run a generated gesture:
1) Run gen_joint_traj_from_cartesian_with_ori.py . It will load the .p extension file previously done, from there do all the IK calls for the points in the trajectory and generate a trajectory message for the controller of the desired arm (right_arm now). Then will send this message to the controller of the arm and wait for it to finish.


TODO: All this manual steps can be done in a script (and probably with a GUI). That will be the next thing to do.

