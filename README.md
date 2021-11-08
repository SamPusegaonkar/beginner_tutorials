# beginner_tutorials

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

A repo containting simple ROS tutorials. This repo contains a [publisher & subscriber tutorial](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29).

# Dependencies

  * I assume that you are using Ubuntu 18.04 with ROS Melodic installed. To install ROS Melodic, please visit this [link](
  http://wiki.ros.org/melodic/Installation/Ubuntu). Do the full-desktop-installation.

  * If you have ROS, cmake (catkin) will already be installed. If you dont, install cmake.

  * Make sure you have a caktin workspace setup. Follow [this](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) for reference.

# How to build this repo
```
  * cd ~/catkin_ws/src
  * git clone https://github.com/SamPusegaonkar/beginner_tutorials/tree/main
  * cd ~/catkin_ws/
  * catkin_make
```
# How to run this repo
```
  * cd ~/catkin_ws/
  * source ./devel/setup.bash
  * roscore
  * Open another 2 terminals & run the first 2 commands in this section again:
  * roslaunch beginner_tutorials listener
  * roslaunch beginner_tutorials talker
```

# How to run this repo - Week10 Branch - Services
```
  * cd ~/catkin_ws/
  * source ./devel/setup.bash
  * roslaunch beginner_tutorials my_launch.launch
  * Open another 2 terminals & run the first 2 commands in this section again:
  * rosservice call /CheckString <string> #This <string> can be DEBUG, WARN, INFO, ERROR, FATAL
  * Open rosrun rqt_console rqt_console # To see the logs
  * Open rosrun rqt_logger_level rqt_logger_level # To change the log type for a node

```


