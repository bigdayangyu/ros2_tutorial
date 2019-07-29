# ros2_tutorial
This tutorial provides references of some commonly used ROS 2 features. Please refer to the [official tutorials](https://index.ros.org/doc/ros2/Tutorials/) for a full introduction of the awesome ROS 2  
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
   * [ROS2 Quality of Service policies](#ros-2-quality-of-service-policies)
   * [ROS2 Lifecycle](#ros2-lifecycle)
5. [Changes in Build System](#ament-build-tool)
   * ament vs catkin
   * CMakelist and packge.xml changes in ROS2 
6. [ROS2 Launch](#ros2-launch-system)
   * basic launch file
   * launch arguments
   * launch parameters 
   * nested launch files 
7. [ROS 2 Networking](#ros-2-networking)
   * [Compare to ROS 1](#what's-new-in-ros2-networking)
   * [How to setup networking on different hosts](#how-to-setup-multiple-hosts-for-ros2)

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
    ros2 pkg -h          # ros2 pkg help
    ```

* ros2 topic
    ```bash
    ros2 topic echo <topic_name>
    ros2 topic list
    ros2 topic -h       # ros2 topic help 
    ```
* ros2 parameter 
    ```bash
    ros2 param list
    ros2 param set node_name Parameter value
    ros2 param set catographer_node use_sim_time true # set the use_sim_time param to be true
    ros2 param -h       # ros2 param help
    ```
* Stop/Start daemon
    ```bash
    ros2 daemon stop
    ros2 daemon start
    ```

## ROS2 New Features

### ROS2 structure 
User Code ---> rclcpp/py ---> rcl ---> rmw ---> rmw_

1. **rclcpp/rclpy**: language specific ROS clients libraries 
2. **rcl**: c Library 
3. **rmw**: ROS middleware interface: hide specific DDS implementation and streamline QoS configuration
4. **rmw_**: DDS adapters 

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

#### Qos predefined profiles 
* rclcpp::SystemDefaultsQoS
* rclcpp::ParametersQoS
* rclcpp::SensorDataQoS
* rclcpp::ServicesQoS

ROS 2 Dashing API changes [link](https://index.ros.org//doc/ros2/Releases/Release-Dashing-Diademata/#rclcpp)

If you have no idea what depth to use and don’t care right now (maybe just prototyping), then we recommend using 10, as that was the default before and should preserve existing behavior.

publishers:
```diff
- pub_ = create_publisher<std_msgs::msg::String>("chatter");
# Assume a history setting of KEEP_LAST with depth 10
+ pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
```
subscribers
```diff
- sub_ = create_subscription<std_msgs::msg::String>("chatter", callback);
# Assume a history setting of KEEP_LAST with depth 10
+ sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
```

Few more examples Publisher: 
```c++
 sample_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud",
      rclcpp::SensorDataQoS());
```

```c++
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
```
Note that `transient_local` is similar to latching in ROS 1.
### ROS2 Lifecycle 
* ROS 2 lifecycle design concepts [link](http://design.ros2.org/articles/node_lifecycle.html)
* ROS 2 lifecycle demo [link](https://github.com/ros2/demos/blob/master/lifecycle/README.rst)

ROS 2 provides a life cycle management system for nodes, which gives user greater control over the sates of ROS system. A managed node allows the launch system to ensure that all components needed for the application have been instantiated correctly before it allows any component to begin executing its behavior. 

One of the example use case would be a system with sensors that have long boosting time. The device driver can be instantiated to configuring state, and start when the sensor is ready to send data. This gives the developer more flexibility of how their packages can be launched, and not worry about stopping the entire system just to change the state of one node. 

Note that the life cycle node does not inherit from the regular `rclcpp::node::Node` but from `rclcpp_lifecycle::LifecycleNode`.


## ROS2 Launch system
ROS Launch system is a tool for easily launching multiple ROS nodes, setting parameters for those nodes, describing the configuration of their system and then execute it as described. launch system will also monitor the state of the processes launched. 

ROS 2 launch system has been significantly changed compare to ROS 1 launch system. The new features are listed as follows: 

* No longer uses XML configuration files, launch files are Python scripts
* Support for Managed Nodes (Lifecycle)
* Node composition from shared libraries

Launch files should be saved as `.launch.py` for example: `my_file.launch.py`.

### launch functions
#### basic launch file
this example launches the kobuki_node which is an excitable node from turtlebot2_drivers package(provided by `turtlebot2_demo` repository)
```python 
import launch 
import launch_ros
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="turtlebot2_drivers",   
            node_executable="kobuki_node",
            node_name="kobuki_node", 
            output="screen")
])
```
#### launch arguments
This example shows ROS2 launches the `static_state_publisher` node from `tf2_ros` package with arguments:
```python
        launch_ros.actions.Node(
            package="tf2_ros",
            node_executable="static_transform_publisher",
            arguments=['0','0', '0.161', '0','0','0','1', 'base_link', 'laser']
            )
```
#### launch parameters 
ROS2 parameters are stored in a parameter yaml files, the following is a parameter file [example](https://index.ros.org/doc/ros2/Tutorials/Node-arguments/) from ros 2 tutorial:
```XML
parameter_blackboard:
    ros__parameters:
        some_int: 42
        a_string: "Hello world"
        some_lists:
            some_integers: [1, 2, 3, 4]
            some_doubles : [3.14, 2.718]
```
To include the parameter setting, you can include the launch parameter in a launch argument. For example, to launch the urg_node with urg_node parameters: 

```python
         launch_ros.actions.Node(
            package="urg_node",
            node_executable="urg_node",
            output="screen",
            arguments=["__params:=/PATH/urg_node.yaml"]
            )
```
#### Nested launch files 
ROS 2 also allows users to launch another launch file inside one launch file
Import libraries: 
```python
import launch.actions 
import launch.launch_description_sources 
```
The nested launch file can be called as the following example: 
```python 
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource([PATH, '/my_file.launch.py']),
        )
```       

#### launch path: 
Import libraries: 
```python
import os
from ament_index_python.packages import get_package_share_directory
```
Finding the package directory: 
```python
# package directory: 
my_pkg_path = ament_index_python.packages.get_package_share_directory(‘PACAKGE_NAME’)

# Combining two path into one: 
my_config_file_path = os.path.join(my_pkg_path, 'configuration_files')
```

Reference Link: [Turtlebot3 demo launch file](https://github.com/ROBOTIS-GIT/turtlebot3/blob/ros2/turtlebot3_bringup/launch/turtlebot3_state_publisher.launch.py)

## Ament Build tool
#### Overview and Background
Reference Link [Ament Tutorial](https://index.ros.org/doc/ros2/Tutorials/Ament-Tutorial/)

`ament` is a meta build system to improve building applications which are split into separate packages. It consists of two major parts:

* a build system (e.g. CMake, Python setup tools) to configure, build, and install a single package
* a tool to invoke the build of individual packages in their topological order

The tool relies on meta information about the packages to determine their dependencies and their build type. This meta information is defined in a manifest file called `package.xml` 

Each package is built separately with its own build system. In order to make the output of one package available to other packages each package can extend the environment in a way that downstream packages can find and use its artifacts and resources. If the resulting artifacts are installed into /usr, for example, it might not be necessary to alter the environment at all since these folders are commonly being searched by various tools.

## ROS 2 Networking
### What's new in ROS2 networking
* ROS 2 UDP uses multicast to allow different node to discover each other and establish communication.
* ROS 2 DDS implementation allows users to specify a domain ID and create a logical barrier to segregate networks.
* If two or more machines are on the same network, and allows multicast, messages can be passed to different machines. 
* No longer needed to set ROS_HOSTNAME, ROS_MASTER, ROS_MASTER_URI

### How to setup multiple hosts for ROS2
* Configure your network to allow multicast 
* Make sure to connect your machines under the same subnet(same network mask)
* Ping two machines between each other 
* Export the same ROS_DOMAIN_ID into different host

To check if multicast is enabled on both machines, first set the ROS_DOMAIN_ID for both machines. ROS_DOMAIN_ID should be same across different hosts, otherwise, nodes cannot automatically discover each other

Open host 1 terminal
```bash
$ export ROS_DOMAIN_ID=<Your Domain ID>    # Domain ID between 0-232
$ source /opt/ros/dashing/setup.bash
$ ros2 multicast receive
 ```

Open host 2 terminal 
```bash
$ export ROS_DOMAIN_ID=<Your Domain ID>    # Domain ID between 0-232
$ source /opt/ros/dashing/setup.bash
$ ros2 multicast send
 ```
If terminal receives message from the second host machine, that means nodes on both machines can communicate with each other. 

Also, if you don't want different machines to interfere each other, just simply set ROS_DOMAIN_ID differently. 

### Reference Links 
* [ROS2 Basics](http://roboscience.org/book/html/ROS/ROS.html)
* [ROS2 Resources](https://github.com/fkromer/awesome-ros2)
* [DDS Slide](https://www.omg.org/news/meetings/workshops/RT-2007/00-T5_Hunt-revised.pdf)
* [ROS2 Presentation](https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/5ce6c85ca4222fe0ccbd5309/1558628472094/2019-05-07_Current_Status_of_ROS_2.pdf) 
* [ROS concepts](https://github.com/ros2/ros2_documentation/blob/master/source/Concepts/Overview-of-ROS-2-concepts.rst)
* [Quality of Service Doc](https://github.com/ros2/ros2_documentation/blob/master/source/Concepts/About-Quality-of-Service-Settings.rst)
