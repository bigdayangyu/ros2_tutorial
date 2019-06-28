## ROS2 demos:
### Issues:
1. kobuki_node: `TransformBroadcaster` vs `StaticTransformBroadcaster`
	* Problem: When use `TransformBroadcaster` to broadcast TF transformation from [odom] to [base_footprint]/[base_link], cartographer_node loose track of TF between [odom] to [gyro_link]

	* Potential cause:
		1. time synchronization: ROS2 turtlebot3_demo using gazebo simulation, if comment out the `use_sim_time` option, same TF error will occur. 

		2. QoS profile settings: the original QoS of publishing `/imu` and `/odom`  setting on turtlebot2_demo was customized as the following 

		```c++
		rmw_qos_profile_t odom_and_imu_qos_profile = rmw_qos_profile_sensor_data;
		odom_and_imu_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		odom_and_imu_qos_profile.depth = 50;
		odom_and_imu_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
		odom_and_imu_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
		```
		However, under [ros2 repo](https://github.com/ros2/cartographer_ros/), the QoS setting for cartographer_node subscribers was updated to the following(recent [issue](https://github.com/ros2/cartographer_ros/issues/28) suggested that`rmw_qos_profile_sensor_data` might be more appropriate)

		```c++
		rmw_qos_profile_default
		```
		After switching from the custom QoS setting to default, cartographer_node works with `StaticTransformBroadcaster` TF messages. Not clear if the QoS setting of `/odom` topic has affected the transform listener of cartographer to receive TF messages from `/tf` topic. 

	* Potential solutions:
		1. time synchronization: Try to setup gazebo simulation for turtlebot2 and use simulation time to check. Or try to compare and understand the 

		2. try different QoS settings. 

2. turtlebot3 ROS2 demo
	*  turtlebot3_cartographer launch system uses `use_sim_time` as a launch parameter. Under the current ROS2 binary release, If a dictionary of parameters is specified, the node name must also be specified. See https://github.com/ros2/launch/issues/139. However, if keep the node name of cartographer_node. The launch system will launch multiple cartographer_node and cause problem. 

	Update on this issue: new commit to the [ros2 repo](https://github.com/ros2/launch_ros/) eliminates the requirement that a node name override be
specified when passing a dictionary of parameters from launch. [This update](https://github.com/ros2/launch_ros/commit/37929eea57f2c7863207a9f808d8b16b73464de9) has not been pushed to binary release. 

	* turtlebot3_cartographer package uses customized cartographer node(ROBOTIS-GIT repo)[https://github.com/ROBOTIS-GIT/cartographer_ros] which was forked from the [ros2 repo](https://github.com/ros2/cartographer_ros/). ROBOTIS uses customized QoS profile for the turtlebot3 drivers. The latest update from ROBOTIS(dashing branch) was Jun 26, 2019 while the latest update from ROS2 was May 17, 2019
