## Specs

* ROS-melodic
* Gazebo-9

## Instructions

* Build Gazebo part (inside `/gazebo`)

```
mkdir build
cd build
cmake ..
make
```

* Launch gazebo

```
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:<dir_path>/gazebo/build
gazebo ../my_robot.world
```

* Build ROS part (inside a catkin-ws)

```
catkin_make
```

* Launch ROS

```
roscore
rosparam set joy_node/dev "/dev/input/jsX"
rosrun joy joy_node
rosrun my_robot main
```
