## Specs

* ROS-melodic
* Gazebo-9
* Ubuntu 18.04

## Instructions

* Build Gazebo part (inside `/gazebo`)

```
mkdir build
cd build
cmake ..
make
```

* Build ROS part (inside `catkin_ws`)

```
catkin_make
```

* Launch app

```
roslaunch my_robot main.launch
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

## Remarks

* Frequent crash of Gazebo app throwing GLXBadDrawable error