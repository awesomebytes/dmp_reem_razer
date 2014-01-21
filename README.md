dmp_reem_razer
===========

1) Capturing position/orientation (orientation pending to be added on calculations) from a razer hydra to capture gestures.

2) Computing dmp in cartersian [x, y, z] and orientation [roll, pitch, yaw] space of the end effector.

3) Generate trajectories using the active dmp for different starting and ending positions.

4) Sending the trajectories to the controllers (calculating collision aware IK's thanks to MoveIt!)


Using the ros package of dmp tests ( http://wiki.ros.org/dmp ) but updated for hydro: https://github.com/makokal/dmp
Also using razer_hydra package updated from: https://github.com/skohlbr/razer_hydra but now in a local fork with changes to the 
default launcher: https://github.com/awesomebytes/razer_hydra
(Current one in debs sometimes gives:
terminate called after throwing an instance of 'std::runtime_error'
  what():  Time is out of dual 32-bit range
 And stops working)


Video of it working available here:
http://www.youtube.com/watch?v=AKAeQqSU-dY


===========
How to use:
(Having Razer Hydra plugged and configured having used the udev rules configuration script).

1) Launch a reem simulation (roslaunch reem_gazebo reem_empty_world.launch)
2) Launch the environment roslaunch dmp_reem_razer launch_environment.launch 
3) Do roscd dmp_reem_razer/src and run ./execute_gesture_from_current_point.py handshake_001.bag
4) Now you can put the right paddle in a position that you like (close to REEM body) and hit RB and the gesture will be computed and executed.


To capture with Razer:
1) Launch a reem simulation (roslaunch reem_gazebo reem_empty_world.launch)
2) Launch roslaunch dmp_reem_razer launch_environment_recording_gesture.launch
2) Run hydra_grab_points.py
3) Press the RB button of the controller to start recording a gesture, when you release it, the gesture ends. This will create a rosbag with the current timestamp with the Pose of the controller.

