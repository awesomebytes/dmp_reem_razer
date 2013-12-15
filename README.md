dmp_reem_razer
===========

1) Capturing position/orientation (orientation pending to be added on calculations) from a razer hydra to capture gestures.

2) Computing dmp in cartersian [x, y, z] space of the end effector.

3) Generate trajectories using the active dmp for different starting and ending positions.

4) Sending the trajectories to the controllers (calculating IK's thanks to MoveIt!)


Using the ros package of dmp tests ( http://wiki.ros.org/dmp ) but upadted for hydro: https://github.com/makokal/dmp

Also plotting with matplotlib the trajectories generated.

