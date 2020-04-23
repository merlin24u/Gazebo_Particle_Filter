## Specs

* ROS-melodic
* Gazebo-9
* Ubuntu 18.04

## Instructions


* Build ROS node and plugins (inside `catkin_ws`) with ROS package named `gazebo_particle_filter`

```
catkin_make
```

* Launch app

```
roslaunch my_robot main.launch
roslaunch my_robot filter.launch
```
