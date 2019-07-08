## Abstract

 This guide gives an overview of ROS2 status with an example of SLAM implementation. ROS2 is an upgrade after one decade since the introduction of ROS (Robotic Operating System) and is still under heavy development at this moment as June 2019. In this guide, we discuss and evaluate some of its new features by implementing SLAM in ROS2 in simulation and on a real robot. These new features are briefly introduced and some are tested in this implementation. A demo with source code is provided in the end of the paper.

## I. Introduction
### 1.1 Robot Operating System Background
Robot Operating System (ROS) is a robotics middleware that was created by Willow Garage and Stanford University and now maintained by Open Source Robotics Foundation(OSRF).  As an open source framework for various robotics software development, ROS provides functions such as hardware abstraction, device control, message passing, package management and libraries for different functionalities. The modularity of ROS allows users to focus on application development rather than spending much effort to reinvent the wheel.

### 1.2 ROS2 Design Background
ROS was originally designed for PR2 use case. PR2 robot works as a standalone robot with excellent network connectivity, also PR2 applications are mostly research based, therefore the early design concept of ROS does not need to consider real-time problems.

Nowadays ROS has gained tremendous popularity in robotics community, and the use cases has grown beyond the scope of academia and scientific research. Many robotics applications such as  industrial robots, outdoor robots(for example driver-less cars), unmanned aerial vehicles(UVA) have become more and more complicated, as a result, those applications have higher demand on the robust real-time performance of the robot operating system. Although ROS1 is still a very popular development tool in the field of robotics, the limitations of the original design have become a driving force of the new ROS2 design. 

With the growing demand of cross operating system platform and real-time functionality from the ROS community, ROS2 development was first announced at ROSCon 2014, and the first alpha version was launched in August 2015. On December 8, 2017, the highly anticipated ROS 2.0 finally released its first official version, Ardent Apalone. As of 2019, the newest version ROS 2 Dashing Diademata was released on May 30.

Compare to ROS1, ROS2 has the following support for robotics applications: 

* *Cross-system platform support*: ROS2 support for Linux, Windows and macOS as well as the real-time operating system(RTOS).
* *Multi-robot system support*: Improved communication system allows robust network performance for multi-robot system 
* *Real-time control*: support to improve the timeliness of a robot control application and overall robot performance
* *Non-ideal networks*:
* *Production environments*:
* *Small embedded platforms*:

### 1.3 ROS2 Communication 
ROS1 uses TCP (Transmission Control Protocol) as its communication protocol. TCP is a connection oriented network, this means that TCP tracks all data sent, requiring acknowledgment for each octet (generally), therefore,  ROS1 has a centralized network configuration which requires a running ROS master to take care of naming and registration services. With the help of the master, ROS nodes could find each other on the network and communicate in a peer-to-peer fashion. In ROS1 setting, all nodes will depend on the central ROS master. When the network becomes lossy and unstable(especially if nodes are distributed on several computers), the communication will not be reliable for real-time applications or multi-robot systems.

ROS2 uses Data Distribution Service (DDS) as the communication middleware. UDP is a Data-Centric-Publish-Subscribe(DCPS) model, and this model will create global data space for individual applications to exchange information. DDS will identify a data object by its topic name and then subscribe to this topic, therefore, DDS does not have a central distributor for all information. The DDS publish-subscribe model avoids complex network programming for distributed applications.  ROS2 provides an abstraction layer of DDS, so users do not need to pay attention to the underlying DDS structure. The ROS2 Middleware Interface(RMW) allows users to choose different Quality of Service(QoS). The real-time publish-subscribe (RTPS) protocol allows ROS2 nodes to automatically find each other on the network, thus there is no need for a ROS2 master. This is an important point in terms of fault tolerance.

## II. Related Work
When ROS2 Bouncy was released, a TurtleBot 2 demo was provided to demonstrate the some popular mapping and localization packages that runs in ROS 2. TurtleBot 2 demo uses Google Cartographer to get maps of the environment, and use AMCL package to localize. TurtleBot 2 demo also provided TurtleBot 2 driver and the Orbbec Astra depth camera sensor driver. At the time this demo was created, ROS2 navigation stack was still under development, therefore, TurtleBot 2 demo uses joystick to manually operate the robot to create maps.  

## III. Method
The objective of this demo is to build a kobuki SLAM and navigation demo on top of the existing TurtleBot 2 demo, and update packages so that the kobuki robot can achieve SLAM and autonomous navigation using the latest ROS 2 Dashing Diademata release. 

1.  Hardware setup 
	1. Robot: Kobuki (turtlebot2)
	2. Sensor: hokuyo laser scanner

2. ROS2 Packages:
	1. mapping: 
		1. cartographer (binary release available)
		2. cartographer-ros 
	2. Visualization:
		1. ros1_bridge (binary release available)
		2. rviz 
	3. Controller and drivers:
		1. teleop_twist_keyboard (binary release available)
		2. urg_node: URG laser scan driver 
		3. tutlebot2_drivers: provide kobuki_node to drive the Kobuki Robot
	3. Simulation: 
		1. Gazebo9 simulation (binary release available)
	4. Navigation 
		1. navigation2 (still under heavy development)
3. Simulation Plan
	1. Setup gazebo for kobuki
	2. Setup laser scan
	3. Run cartographer in simulation 
	4. After getting map, try the navigation stack

## IV. Results
Describe your results in teleoperation, mapping, localization, and navigation.V. DiscussionIntroduce your thoughts of new features ROS2 while doing the implementation. Describe the problems you encountered and your understanding of the solution.

## VI. Conclusion
Give your conclusion of the completeness of ROS2 in SLAM and navigation.

## VII. Reference

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
