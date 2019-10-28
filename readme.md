# ROS Beginner tutorials - ENPM808X
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Project Overview

This project shows the development of basic publisher and subscriber nodes in ROS. The process is followed as given on the ROS wiki: http://wiki.ros.org/ROS/Tutorials

The model has two nodes:
1. Talker - src/talker.cpp (Publisher)
2. Listener - src/listener.cpp (Subscriber)


## Dependencies
The project has the following dependencies.

1. ROS Kinetic ([Installation](http://wiki.ros.org/kinetic/Installation))
2. catkin ([Installation](http://wiki.ros.org/catkin#Installing_catkin))

## Build steps
 
- To build the given project, create and build the catkin workspace by following the given steps:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash
cd src/
git clone --recursive https://github.com/rohansingh42/beginner_tutorials.git
cd ~/catkin_ws/
catkin_make
```

- NOTE: For running command from each new terminal, source the devel/setup.bash file in every new terminal before executing any ros command, or add the following line to your _~/.bashrc_ once
```
source ~/catkin_ws/devel/setup.bash
```

## Running the project

To run the publisher and subscriber nodes, follow the given steps (after sourcing _devel/setup.bash_ according to above note):

- In a new terminal,
```
roscore
```
- To run the Publisher node, type in another terminal,
```
rosrun beginner_tutorials talker
```

- To run Subscriber node, type in another terminal,
```
rosrun beginner_tutorials listener
```

- To just check the published message (without running any subscriber), type in a new terminal,
```
rostopic echo /chatter
```