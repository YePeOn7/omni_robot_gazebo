# Three-Wheeled Omnidirectional Gazebo Simulation

## Introduction

This repository contains a Gazebo simulation for a 3-wheeled omnidirectional robot, useful for robotics research and simulation.

### Tested on

- Ubuntu 20.04
- ROS Netic

## How to run

Please clone this repository to your catkin workspace. The following assume your catkin work space placed on your home directory with named **catkin_ws**

```bash
cd ~/catkin_ws/src
git clone https://github.com/YePeOn7/omni_robot_gazebo.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

After that you can run the simulation as follow

```bash
cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch omni_robot_gazebo omni_world.launch
```

## Special Thanks

- [robot mania](https://www.youtube.com/@robotmania8896) (original author)
- [KairongWu](https://github.com/KairongWu)
