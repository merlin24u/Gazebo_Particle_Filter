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

* Launch gazebo (inside `/gazebo`)

```
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/my_robot/gazebo/build
gazebo ../my_robot.world
```

* Build ROS part (inside catkin_ws)

```
catkin_make
```

* Launch ROS

```
roscore
rosparam set joy_node/dev "/dev/input/js0"
rosrun joy joy_node
rosrun my_robot main
```
