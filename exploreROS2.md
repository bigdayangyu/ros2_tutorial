## Abstract

 This guide gives an overview of ROS2 status with an example of SLAM implementation. ROS2 is an upgrade after one decade since the introduction of ROS (Robotic Operating System) and is still under heavy development at this moment as June 2019. In this guide, we discuss and evaluate some of its new features by implementing SLAM in ROS2 in simulation and on a real robot. These new features are briefly introduced and some are tested in this implementation. A demo with source code is provided in the end of the paper.

## I. Introduction
ROS2 history and new features like new network setup, realtime operating system, security etc. (Briefly mentioned)ROS2 packages status like SLAM (cartographer), sensor driver (lidar, camera etc), navigation Brief discussion on the implementation of SLAM and navigation in ROS2, like what is avaiable, what is missing, how complete is ROS2 in a whole navigation setup.

## II. Related Work

Early efforts with demos in turtlebot and recent changes. What solution is used to setup a navigation and SLAM system on turtlebot.

## III. Method
Define your implementation of SLAM and navigation with ROS2 in simulation and on real robot.Define your solution, like hardware platform setup, software components, user interface etc.

## IV. Results
Describe your results in teleoperation, mapping, localization, and navigation.V. DiscussionIntroduce your thoughts of new features ROS2 while doing the implementation. Describe the problems you encountered and your understanding of the solution.

## VI. Conclusion
Give your conclusion of the completeness of ROS2 in SLAM and navigation.

## VII. Reference

1.  Hardware setup 
	1. Robot: Kobuki (turtlebot2)
	2. Sensor: hokuyo laser scanner
2. Controller: keyboard 
3. ROS2 Packages:
	1. mapping: 
		1. urg_node
		2. kobuki_node (tutlebot2_drivers)
		3. cartographer
	2. Visualization:
		1. ros1_bridge
		2. rviz 

	3. Simulation: 
		1. Gazebo9 simulation
	4. Navigation 
		1. navigation2 ?
4. Simulation Plan
	1. Setup gazebo for kobuki
	2. Setup laser scan
	3. Run cartographer in simulation 
	4. After getting map, try the navigation stack