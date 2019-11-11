# ROS Beginner tutorials - ENPM808X
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Project Overview

This project shows the development of basic publisher and subscriber nodes in ROS. The process is followed as given on the ROS wiki: http://wiki.ros.org/ROS/Tutorials

The model has two nodes:
1. Talker - src/talker.cpp (Publisher and Service Node)
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

- To run all nodes with Publishing frequency as argument,
```
roslaunch beginner_tutorials allNodes.launch pubHz:=<Publishing frequency>
```

- To run all nodes while recording all topics, type in a new terminal,
```
roslaunch beginner_tutorials allNodes.launch record:=true
```
Output will be saved in the _results_ folder. Make flag __record__ param flag "false" or don't use it at all to not record the data.

## Using the service

- To use the ConcatStrings service, type in a new terminal while the talker node is running,
```
rosservice call /concatStringService "first: '<string 1>' second: '<string 2>'" 
```
The talker node will publish the concatenated string.

## Viewing data

- To check the published message (without running any subscriber), type in a new terminal,
```
rostopic echo /chatter
```

- To see frame output, type in a new terminal while the talker node is running,
```
rosrun tf tf_echo /world /talk
```

- To see frame tree, type in a new terminal while the talker node is running,
```
rosrun rqt_tf_tree rqt_tf_tree
```

- To replay the recorded bag file and see the result in listener node, run the listener node, then type in a new terminal,
```
rosbag play /results/<name of bagfile>
```
Assuming tha command is run from base directory. If not, give relative path to bag file from current directory.

## Running Tests

- To run tests during building,  
```
cd ~/catkin_ws/
catkin_make run_tests_beginner_tutorials
```

- To run test seperately, after building the system normally, type in a terminal,
```
rostest beginner_tutorials allTests.launch
```
