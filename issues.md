## Turtlebot2_demo:
### known issue:
1. kobuki_node: `TransformBroadcaster` vs `StaticTransformBroadcaster`
	* problem: When use `TransformBroadcaster` to broadcast TF transformation from [odom] to [base_footprint]/[base_link], cartographer_node loose track of TF between [odom] to [gyro_link]

	* Potential cause:
		1. time synchronization: ROS2 turtlebot3_demo using gazebo simulation, if comment out the `use_sim_time` option, same TF error will occur. 

		2. QoS profile settings: the original QoS setting on turtlebot2_demo was customized as the following 
		```c++

			rmw_qos_profile_t odom_and_imu_qos_profile = rmw_qos_profile_sensor_data;
			odom_and_imu_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
			odom_and_imu_qos_profile.depth = 50;
			odom_and_imu_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
			odom_and_imu_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
		```