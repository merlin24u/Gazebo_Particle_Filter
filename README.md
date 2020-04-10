## Specs

* ROS-melodic
* Gazebo-9
* Ubuntu 18.04

## Instructions

* System (modify `~/.bashrc`)

```
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/my_robot/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/src/my_robot/gazebo/build
```

* Build Gazebo part (inside `/gazebo`)

```
mkdir build
cd build
cmake ..
make
```

* Build ROS part (inside catkin_ws)

```
catkin_make
```

* Launch app

```
roslaunch my_robot my_robot.launch
```

* Launch gazebo seperatly (inside `/gazebo`)

```
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/my_robot/gazebo/build
gazebo -s libgazebo_ros_api_plugin.so my_robot.world
```

* Launch ROS seperatly

```
roscore
rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node
rosrun my_robot main
```
