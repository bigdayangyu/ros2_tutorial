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

## Reference Knowledge 

### ROS2 Background 
Robot Operating System (ROS) is a robotics middleware that was created by Willow Garage and Stanford University, ROS provide a frameworks for robotics software development. ROS provides functions such as hardware abstraction, device control, message passing, package management and libraries for different functionalities. 

Reference Link: [why ROS2](http://design.ros2.org/articles/why_ros2.html)

ROS1 uses TCPROS/UDPROS as its communication system, so ROS1 has a centralized network configuration which requires a running ROS master to take care of naming and registration services. With the help of the master, ROS nodes could find each other on the network and communicate in a peer-to-peer fashion. In ROS1 setting, all nodes will depend on the central ROS master. When the network becomes lossy and unstable(especially if nodes are distributed on several computers), the communication will not be reliable for real-time application or multi-robot system. 

Reference Link: [Why ROS2 does not need a master](https://arxiv.org/pdf/1905.09654.pdf)

With the growing demand of cross operating system platform and real-time functionality from the ROS community, ROS2 was developed by Open Robotics Foundation(OSRF). 

Compare to ROS1, ROS2 uses Data Distribution Service (DDS) as the communication middleware. ROS2 provides a Middleware Interface(RMW) that allows users to choose different Quality of Service(QoS). The real-time publish-subscribe (RTPS) protocol allows ROS2 nodes to automatically find each other on the network, thus there is no need for a ROS2 master. This is a important point in terms of fault tolerance.


### ROS2 Cartographer


### ROS Navigation stack 
To navigate a robot we need 

* a map 
* localization module
* path planning module  

Additional requirement for changing environment 

* Obstacle-Detection/Avoidance
* Local Map Refinement, based on the most recent sensor reading.

1. Global Planner
	
	The global planner uses a prior knowledge of the environment(a map of the environment) to find a optimal path for the robot to get to a desired coordinate. The common algorithms are Dijkstra, A* etc. 

2. Local Planner
	
	Once a global map has been generated, the local planner will generates sets of feasible local trajectories that are safe and collision-free so the robot can take a fast motion to reach the goal. Common local planners are Dynamic Window Approach(DWA), Time Elastic Bands (TEB). Those algorithms will sample around the robot and and generate simulated trajectories, and then select the highest-scored trajectory based on expected outcome. Then local planner will send the velocity command to the robot and repeat until the goal has been reached. 

	[reference](https://www.hindawi.com/journals/jat/2018/6392697/)
