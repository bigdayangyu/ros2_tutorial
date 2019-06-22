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
5. CMakelist and packge.xml changes in ROS2 

## Install ROS2 dashing 
[Dashing Linux Install](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/)

## Workspace and Packages
### Source the ros2 enviornment 
```bash
$ source /opt/ros/$ROS_DISTRO/setup.bash
```
### Create a new ros2 workspace
```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws
```
And then put some packages into the src folder 
### Create a new ros2 package
 1. Using command 
 ```bash 
 $ ros2 pkg create --<package name> [deps]
 ```
 2. Create Pacakge Manually
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

* Stop daemon
    ```bash
    ros2 daemon stop
    ```

## ROS2 New Features

### ROS bridge between ROS1 and ROS2
Reference link: [ros2_bridge](https://github.com/ros2/ros1_bridge/blob/master/README.md#build-the-bridge-from-source)

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

#### QoS policies
Reference Link: [QoS policies](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)

Current QoS profile settings: 
* History
  * Keep last: only store up to N samples, configurable via the queue depth option.
  * Keep all: store all samples, subject to the configured resource limits of the underlying middleware.
* Depth
  * Size of the queue: only honored if used together with “keep last”.
* Reliability
  * Best effort: attempt to deliver samples, but may lose them if the network is not robust.
  * Reliable: guarantee that samples are delivered, may retry multiple times.
* Durability
  * Transient local: the publisher becomes responsible for persisting samples for “late-joining” subscribers.
  * Volatile: no attempt is made to persist samples.


https://github.com/ros2/ros2_documentation/blob/master/source/Releases/Release-Dashing-Diademata.rst