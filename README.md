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
 2. Show all output on the console after a package has finished
 ```bash
 $ colcon <verb> --event-handlers console_cohesion+
 ```
 3. Build only a single package (or selected packages)
 ```bash
 $ colcon build --packages-select <name-of-pkg>
 $ colcon build --packages-select <name-of-pkg> <name-of-another-pkg>
 ```
 4. Build selected packages including their dependencies
 ```bash
 $ colcon build --packages-up-to <name-of-pkg>
 ```
 5. Rebuild packages which depend on a specific package
 ```bash
 $ colcon build --packages-above <name-of-pkg>
  ```
 6. To ignore the package while building 
 insert AMENT_IGNORE into the package 
 
 6. Clean cmake cache
```bash
  colcon build --symlink-install --packages-select turtlebot2_drivers --cmake-clean-cache
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
1. Broadcasting Transforms
Reference Link: [tf2_ros](http://wiki.ros.org/tf2_ros)
* Broadcast Transformation: 
  * `tf2_ros::TransformBroadcaster() ` constructor
  * `tf2_ros::TransformBroadcaster::sendTransform`to send transforms 
* Broadcast Static Transformation 
  * `tf2_ros::StaticTransformBroadcaster()`, constructor,
  * `tf2_ros::StaticTransformBroadcaster::sendTransform` to send static transforms 
### ROS 2 Quality of Service policies
Reference link: [ROS2 QoS design](https://design.ros2.org/articles/qos.html)
 
