# ros2_tutorial

### source the ros2 enviornment 
```bash
$ source /opt/ros/$ROS_DISTRO/setup.bash
```
### Create a new ros2 workspace
```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws
```
And then put some packages into the src folder 
### Build the workspace
```bash
$ colcon build --symlink-install
```
#### colcon build options 
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
 ### Create a new ros2 package
 1. Using command 
 ```bash 
 $ ros2 pkg create --<package name> [deps]
 ```
 2. Create Pacakge Manually
* Create c++ package
    * The package should contain a file named ``package.xml`` provides meta information about the package
    * The package should contain a file named ``CMakeLists.txt``, provide information about the package and dependencies
    Sample ``package.xml``


 
