# Staubli VAL3 driver

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![Github Issues](https://img.shields.io/github/issues/ros-industrial/staubli_val3_driver.svg)](http://github.com/ros-industrial/staubli_val3_driver/issues)

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)


## Overview

This repository contains the `staubli_val3_driver` package which provides a set of VAL3 libraries and an application which together implement a [simple_message][] compatible server implementation.
Together with the nodes in [industrial_robot_client][], this server can be used as a ROS 1 driver that allows motion control of Staubli CS8/CS8C controlled robots, by exposing a [FollowJointTrajectory][] [action][] server, which is compatible with MoveIt and other nodes that implement an action client.


## Documentation

Refer to the `staubli_val3_driver` [readme](./staubli_val3_driver/README.md) for more information on requirements, setup and use.


## Compatibility

The current version of the driver is compatible with CS8/CS8C controllers only.
Future work is planned to extend this to support CS9 controllers as well.



[simple_message]: http://wiki.ros.org/simple_message
[industrial_robot_client]: http://wiki.ros.org/industrial_robot_client
[FollowJointTrajectory]: http://docs.ros.org/api/control_msgs/html/action/FollowJointTrajectory.html
[action]: http://wiki.ros.org/actionlib
