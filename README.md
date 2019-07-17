# ros2_tutorial

## Table of Contents
1. [Install ROS2 dashing ](#install-ros2-dashing)
2. [Workspace and Packages](#workspace-and-packages)
   * [Create ROS2 Workspace](#create-a-new-ros2-workspace)
   * [Create ROS2 Package](#create-a-new-ros2-package)
   * [Build ROS2 Packages using Colcon Build](#build-the-workspace)
3. [Run ROS2 command](#ros2-command)
4. [New Features in ROS2](#ros2-new-features)
   * [ROS1-ROS2 bridge](#ros-bridge-between-ros1-and-ros2)
   * [TF2](#tf2)
   * [ROS 2 Quality of Service policies](#ros-2-quality-of-service-policies)
5. Changes in Make system 
   * ament vs catkin
   * CMakelist and packge.xml changes in ROS2 
6. ROS2 Launch
   * Finding Path 
   * Node Names: launching multiple node with same node name problem
   * Launch argument 

## Install ROS2 dashing 
[Dashing Linux Install](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)

## Workspace and Packages
### Source the ROS2 Environment 
```bash
$ source /opt/ros/$ROS_DISTRO/setup.bash
```
### Create a new ROS2 workspace
```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws
```
And then put some packages into the src folder 
### Create a New ROS2 Package
 1. Using command 
 ```bash 
 $ ros2 pkg create --<package name> [deps]
 ```
 2. Create Package Manually
* Create C++ package
    * The package should contain a file named ``package.xml`` provides meta information about the package
    * The package should contain a file named ``CMakeLists.txt``, provide information about the package and dependencies
    Sample ``package.xml``
* C++ demo package [demo_nodes_cpp](https://github.com/ros2/demos/tree/dashing/demo_nodes_cpp)
    * Publisher/Subscriber
    * Launch 
    * Service 
* Create Python package
    * The package should contain a file named ``package.xml`` provides meta information about the package
    * The package should contain a file named ``setup.py``, provide information about the package
* Python demo package [rclpy](https://github.com/ros2/examples/tree/dashing/rclpy)
    * Publisher/Subscriber
    * Launch 
    * Service 

### Build the workspace
```bash
$ colcon build --symlink-install
```

#### Colcon Build options 
Reference Page: [colcon documentation](https://buildmedia.readthedocs.org/media/pdf/colcon/latest/colcon.pdf)
1. Show all output immediately on the console
 ```bash 
 $ colcon build --event-handlers console_direct+
 ```

2. Build only a single package (or selected packages)
 ```bash
 $ colcon build --packages-select <name-of-pkg> <name-of-another-pkg>
 ```

3. Build selected packages including their dependencies
 ```bash
 $ colcon build --packages-up-to <name-of-pkg>
 ```

4. To ignore the package while building 
 insert AMENT_IGNORE into the package 
 
5. Clean cmake cache
  ```bash
  $ colcon build --symlink-install --packages-select turtlebot2_drivers --cmake-clean-cache
  ```

## ROS2 Command
* run ros2 node 
  ```bash
  ros2 run <package_name> <executable>
  ```

* ros2 launch
    * launch file should be saved as `.launch.py`. eg. `my_file.launch.py`
    ```bash 
    ros2 launch <package_name> <launch_file>
    ```

* ros2 pkg 
    ```bash
    ros2 pkg list
    ros2 pkg prefix <package_name>
    ```

* ros2 topic
    ```bash
    ros2 topic echo <topic_name>
    ros2 topic list
    ```
* ros2 parameter 
    ```bash
    ros2 param list
    ros2 param set node_name Parameter value
    ros2 param set catographer_node use_sim_time true
    ```

* Stop daemon
    ```bash
    ros2 daemon stop
    ```

## ROS2 New Features

### ROS2 structure 
User Code ---> rclcpp/py ---> rcl ---> rmw ---> rmw_

1. *rclcpp/rclpy* language specific ROS clients libraries 
2. *rcl* c Library 
3. *rmw* ROS middleware interface: hide specific DDS implementation and streamline QoS configuration
4. *rmw_* DDS adapters 

### ROS bridge between ROS1 and ROS2
Reference link: [ros1_bridge](https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source)

1. Dynamic bridge vs Static bridge

  **Dynamic bridge** can automatically open bridges while listening to topics from 
  both sides. However, inconsistency of received message can close the dynamic bridge and lead to a temporary loss of transfer. 

  **Static bridges** can be modified so that they could pass custom messages of a single topic in one direction only. The performance of Static bridges doesn’t depend on the periodical consistency of the messages. 

2. How to write custom pairs of messages for ros1_bridge 

### TF2 

#### Background of TF2 design 
1. Why `/tf_static`  
    Reference Link: [TF2 Design](http://wiki.ros.org/tf2/Design)

    tf messages do not deal with low bandwidth networks well. Within a single well connected network tf works fine, but as you transition to wireless and lossy networks with low bandwidth the tf messages start to take up a large fraction of the available bandwidth. This is partially due to the many to many nature and partially due to the fact that there is no way to choose the desired data rate for each consumer.

    **Solution**: Add support for `/tf_static` topic which will only publish latched topics. When the subscriber is not yet established, publisher will store the latest message. Once subscriber being established, the subscriber will not miss the message. 

#### Feature summary 

Reference Link: [tf2_ros](http://wiki.ros.org/tf2_ros)

1. Broadcasting Transforms
* Broadcast Transformation:
  
  Publish to `/tf` topic

  * `tf2_ros::TransformBroadcaster() ` constructor
  * `tf2_ros::TransformBroadcaster::sendTransform`to send transforms 
* Broadcast Static Transformation : 

  Publish to `/tf_static` topic.

  Use for "latching" behavior when transforms that are not expected to change.
  * `tf2_ros::StaticTransformBroadcaster()`, constructor,
  * `tf2_ros::StaticTransformBroadcaster::sendTransform` to send static transforms 

2. Transform Listener

    After the TransformListener object is created, it starts receiving tf2 transform over the wire, and buffers them for up to 10 seconds.The TransformListener object should be scoped to persist otherwise it's cache will be unable to fill and almost every query will fail. The common method is to make the TransformaListener object a member variable of a class.  


### ROS 2 Quality of Service policies

#### Overview and Background
Reference link: [ROS2 QoS design](https://design.ros2.org/articles/qos.html)

ROS1 uses TCP as the underlying transport,which is unsuitable for lossy networks such as wireless links. ROS2 uses UDP as its transport, which offers a variety of Quality of Service (QoS) policies. This new feature benefits from the flexibility of controlling the right Quality of Service profile needed for a node to expect and act accordingly(In real-time computing systems where the right Quality of Service profile is needed to meet deadlines).
 
Reference link: [Difference Between TCP and UDP ](https://enterprise.netscout.com/edge/tech-tips/difference-between-tcp-and-udp)

TCP (Transmission Control Protocol) is connection oriented, whereas UDP (User Datagram Protocol) is connection-less. This means that TCP tracks all data sent, requiring acknowledgment for each octet (generally). UDP does not use acknowledgments at all, and is usually used for protocols where a few lost datagrams do not matter.

Reference Link:[Why ROS2 does not need a master](https://arxiv.org/pdf/1905.09654.pdf)

ROS1 uses TCP, so ROS1 has a centralized network configuration which requires a running ROS master to take care of naming and registration services. With the help of the master, ROS nodes could find each other  on the network and communicate in a peer-to-peer fashion. In ROS1 setting, all nodes will depend on the central ROS master. When the network becomes lossy and unstable(especially if nodes are distributed on several computers), the communication will not be reliable for real-time application. 

ROS2 uses [Data Distribution Service](https://en.wikipedia.org/wiki/Data_Distribution_Service) (DDS) as the communication middleware. ROS2 provides a Middleware Interface(RMW) that allows users to choose different Quality of Service(QoS). The real-time publish-subscribe (RTPS) protocol allows ROS2 nodes to automatically find each other on the network, thus there is no need for a ROS2 master. This is a important point in terms of fault tolerance.

#### QoS policies
Reference Link: [QoS policies](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)

<!-- 1. Using QoS to send data
    * Continuous Data: 
      * **best-effort** : Constantly updating data 
      * **keep-last**   : Sensor data, last value is best
      * **ownership, deadline**: Seamless fail over
    * State Information
      * **durability** : Occasionally changing persistent data
      * **history** : Recipients need latest and greatest
    * Alarms & Events
      * **liveliness** : Asynchronous messages
      * **reliability** : Need confirmation of delivery -->
**1. Current available ROS2 QoS policy options:**

* Deadline
  * A *DataWriter* and a *DataReader* must update data at least once every deadline period. 

* History
  * This controls whether the data transport should deliver only the most recent value, all intermediate values, o deliver something in between, which is configurable via the `depth`(size of the queue) option
    * *KEEP_LAST*: only store up to N samples, configurable via the queue depth option.
    * *KEEP_ALL* : store all samples, subject to the configured resource limits of the underlying middleware.

* Depth
  * Size of the queue: only honored if used together with “keep last”.

* Reliability
  * *BEST_EFFERT*: data transport is executed ad soon as possible. But may lose them if the network is not robust.
  * *RELIABLE*: missed samples are retransmitted, therefore, sample delivery is guaranteed delivered. may retry multiple times.

* Durability
  * With this policy, the service attempts to keep several samples so that they can be delivered to any potential late-joining *DataDreader*. The number of saved samples depends on HISTORY. This option has several values, such as the following:
      * *TRANSIENT_LOCAL*: the publisher becomes responsible for persisting samples for “late-joining” subscribers.
      * *VOLATILE*: no attempt is made to persist samples.

**2. The currently-defined QoS profiles for different use case:**

Reference Link [RMW QoS Profile Header File](https://github.com/ros2/rmw/blob/release-latest/rmw/include/rmw/qos_profiles.h)

* Default QoS settings for publishers and subscribers: `rmw_qos_profile_default`
  * In order to make the transition from ROS 1 to ROS 2, exercising a similar network behavior is desirable. By default, publishers and subscribers are reliable in ROS 2, have volatile durability, and “keep last” history.
* Services: `rmw_qos_profile_services_default`
  * In the same vein as publishers and subscribers, services are reliable. It is especially important for services to use volatile durability, as otherwise service servers that re-start may receive outdated requests. While the client is protected from receiving multiple responses, the server is not protected from side-effects of receiving the outdated requests.
* Sensor data: `rmw_qos_profile_sensor_data`
  * For sensor data, in most cases it’s more important to receive readings in a timely fashion, rather than ensuring that all of them arrive. That is, developers want the latest samples as soon as they are captured, at the expense of maybe losing some. For that reason the sensor data profile uses best effort reliability and a smaller queue depth.
* Parameters: `rmw_qos_profile_parameter_events`
  * Parameters in ROS 2 are based on services, and as such have a similar profile. The difference is that parameters use a much larger queue depth so that requests do not get lost when, for example, the parameter client is unable to reach the parameter service server.
* System default `rmw_qos_profile_system_default`
 * This uses the system default for all of the policies.

### Qos APIs
* rclcpp::SystemDefaultsQoS
* rclcpp::ParametersQoS
* rclcpp::SensorDataQoS
* rclcpp::ServicesQoS


ROS 2 Dashing API changes [link](https://index.ros.org//doc/ros2/Releases/Release-Dashing-Diademata/#rclcpp)

If you have no idea what depth to use and don’t care right now (maybe just prototyping), then we recommend using 10, as that was the default before and should preserve existing behavior.


publishers:
```diff
- pub_ = create_publisher<std_msgs::msg::String>("chatter");
+ pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
```
subscribers
```diff
- sub_ = create_subscription<std_msgs::msg::String>("chatter", callback);
+ sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
```

Few examples:
Publisher: 
```c++
 sample_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud",
      rclcpp::SensorDataQoS());
```

```c++
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
```
### ROS2 Launch system
Reference Link: [Turtlebot3 demo launch file](https://github.com/ROBOTIS-GIT/turtlebot3/blob/ros2/turtlebot3_bringup/launch/turtlebot3_state_publisher.launch.py)

### Ament Build tool
#### Overview and Background
Reference Link [Ament Tutorial](https://index.ros.org/doc/ros2/Tutorials/Ament-Tutorial/)

`ament` is a meta build system to improve building applications which are split into separate packages. It consists of two major parts:

* a build system (e.g. CMake, Python setup tools) to configure, build, and install a single package
* a tool to invoke the build of individual packages in their topological order

The tool relies on meta information about the packages to determine their dependencies and their build type. This meta information is defined in a manifest file called `package.xml` 

Each package is built separately with its own build system. In order to make the output of one package available to other packages each package can extend the environment in a way that downstream packages can find and use its artifacts and resources. If the resulting artifacts are installed into /usr, for example, it might not be necessary to alter the environment at all since these folders are commonly being searched by various tools.



https://github.com/ros2/ros2_documentation/blob/master/source/Releases/Release-Dashing-Diademata.rst
### Reference Links 
* ROS2 Basics](http://roboscience.org/book/html/ROS/ROS.html)
* [ROS2 Resources](https://github.com/fkromer/awesome-ros2)
* [DDS Slide](https://www.omg.org/news/meetings/workshops/RT-2007/00-T5_Hunt-revised.pdf)
* [ROS2 Presentation](https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/5ce6c85ca4222fe0ccbd5309/1558628472094/2019-05-07_Current_Status_of_ROS_2.pdf) 

