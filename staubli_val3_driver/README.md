# ROS-Industrial driver (server) for Staubli robots

## Overview

This ROS-I driver was developed in Staubli's VAL 3 language for use with 6-axis Staubli robot manipulators.

It is advisable to try this driver on the emulator in [Staubli Robotics Suite (SRS)](https://www.staubli.com/en-us/robotics/product-range/robot-software/pc-robot-programming-srs/) first.


## Requirements

* Staubli 6-axis robot manipulator
* Staubli CS8/CS8C controller
* VAL3 version s7.7.2 or greater
  * this is very important, since this implementation uses return values of `sioGet()`
    only available from s7.7.2 onwards
* Staubli Robotics Suite 2019 (not required but strongly recommended)

## Installation

Installing the driver to a Staubli controller simply consists of transferring the contents of the `val3` folder to the controller itself.

### Clone this repository

Clone [staubli_val3_driver](https://github.com/ros-industrial/staubli_val3_driver) into the `src` space of your workspace:

```shell
git clone https://github.com/ros-industrial/staubli_val3_driver.git
```

### Transfer driver to Staubli CS8/CS8C controller

There are 2 ways of transferring the VAL 3 application to the controller:

1. Copy the contents of `val3` folder onto a USB memory stick (<2GB given the CS8/CS8C limitations), plugging the stick into the controller and using the teach pendant to copy the folders

1. Use the Transfer Manager in SRS to copy the VAL 3 applications to the controller. (Home -> Controller -> Transfer Manager)

### Open the VAL3 application with Staubli SRS

Although it is possible to edit the source files with any text editor (they are essentially XML files), it is advisable to use [Staubli Robotics Suite](https://www.staubli.com/en-us/robotics/product-range/robot-software/pc-robot-programming-srs/):

* Copy contents of folder `val3` into the `usrapp` folder of the Staubli cell
* Open the `ros_server` VAL3 project located inside the `ros_server` folder

SRS offers autocompletion, syntax highlighting and syntax checking, amongst other useful features, such as a Staubli controller/teach pendant emulator. 

## Usage

### Load driver from (real or emulated) teach pendant

From `Main menu`:

1. Application manager --> Val3 applications
2. +Disk --> ros_server

### Configuration

The TCP sockets on the CS8/CS8C controller/emulator must be configured prior to use, otherwise a runtime error will be generated and the driver will not work.

Two sockets (TCP Servers) are required. From `Main menu`:

1. Control panel --> I/O --> Socket --> TCP Servers
2. Configure two sockets
   * Name: Feedback, Port: 11002, Timeout: -1, Delimiter: 13, Nagle: Off
   * Name: Motion, Port: 11000, Timeout: -1, Delimiter: 13, Nagle: Off

### Run the driver (ROS-I server)

Check that:

1. The contents of the `val3` folder (both `ros_server` and `ros_libs` folders)
have been transferred to the Staubli controller
2. The VAL3 application `ros_server` has been loaded
3. Both TCP Server sockets have been configured properly

Press the `Run` button, ensure that `ros_server` is highlighted, then press `F8` (Ok).

Depending on which working mode the controller is in, arm power may need to be enabled manually (a message will pop up on the screen).
Once arm power is enabled, the robot will only move if the `Move` button has been pressed (or is kept pressed if in manual mode).

### Run the industrial_robot_client node (ROS-I client)

The `indigo-devel` branch provides launch files (within the `staubli_val3_driver` ROS package). Simply run:

```shell
roslaunch staubli_val3_driver robot_interface_streaming.launch robot_ip:=<controller IP address>
```

## Bugs, suggestions and feature requests

Please report any bugs you may find, make any suggestions you may have and request new features you may find useful via GitHub.
