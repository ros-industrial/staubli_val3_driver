# ROS-Industrial driver (server) for Staubli robots

## Overview

This ROS-I driver was developed in Staubli's VAL 3 language for use with 6-axis
Staubli robot manipulators.

It is advisable to try this driver on Staubli's emulator in Staubli Robotics Suite (SRS) first.


## Requirements

* Staubli 6-axis robot manipulator
* Staubli CS8/CS9 controller
* VAL 3 version s7.7.2 or greater
  * this is very important, since this implementation uses return values of `sioGet()`
    only available from s7.7.2 onwards
* Staubli Robotics Suite 2019 (not required but strongly recommended)

## Installation

Installing the driver to a Staubli controller simply consists of transferring the
contents of the `val3` folder to the controller itself.

### Clone this repository

Clone branch `master` of [staubli_val3_driver](https://github.com/ros-industrial/staubli_val3_driver):

```shell
git clone https://github.com/ros-industrial/staubli_val3_driver
```

### Transfer driver to Staubli controller

There are multiple ways of transferring VAL 3 applications to the controller:

1. Copy the contents of `val3` folder onto a USB memory stick (<2GB if using CS8), 
plugging the stick into the controller and using the teach pendant to copy the folders

2. Use the Transfer Manager in SRS to copy the contents of `val3` folder to the controller. (Home -> Controller -> Transfer Manager)

2. Use an FTP software to copy the contents of `val3` folder to the controller.

### Open the VAL 3 application with Staubli SRS

Although it is possible to edit the source files with any text editor (they are
essentially XML files), it is advisable to use Staubli Robotics Suite:

* Copy contents of folder `val3` into the `usrapp` folder of the Staubli cell
* Open the `ros_server` VAL 3 appplication located inside the `ros_server` folder

SRS offers autocompletion, syntax highlighting and syntax checking, amongst other
useful features, such as a Staubli controller/teach pendant emulator.


## Usage

### Load driver from (real or emulated) teach pendant

From `Main menu`:

1. Application manager --> Val3 applications
2. +Disk --> ros_server

### Configuration

The TCP sockets on the CS8/CS9 controller/emulator must be configured prior to using
the driver, otherwise a runtime error will be displayed on the teach pendant and
the driver will not work.

Two sockets (TCP Servers) are required.

#### CS8

 From `Main menu`:

1. Control panel --> I/O --> Socket --> TCP Servers
2. Configure two sockets
   * Name: Feedback, Port: 11002, Timeout: -1, End of string: 13, Nagle: Off
   * Name: Motion, Port: 11000, Timeout: -1, End of string: 13, Nagle: Off

#### CS9

 From `Home`:

1. IO --> Socket --> TCP Servers --> "+"
2. Configure two sockets
   * Name: Feedback, Port: 11002, Timeout: -1, End of string: 13, Nagle: Off
   * Name: Motion, Port: 11000, Timeout: -1, End of string: 13, Nagle: Off

### Run the driver (ROS-I server)

Check that:

1. The contents of the `val3` folder (both `ros_server` and `ros_libs` folders)
have been transferred to the Staubli controller
2. The VAL 3 application `ros_server` has been loaded
3. Both TCP Server sockets have been configured properly

#### CS8

Press the `Run` button, ensure that `ros_server` is highlighted,
then press `F8` (Ok).

#### CS9

VAL# --> Memory --> select `ros_server` --> ▶

Notice that depending on which mode of operation is currently active, the motors
may need to be enabled manually (a message will pop up on the screen). Likewise,
the robot will only move if the `Move` button has been pressed (or is kept pressed
if in manual mode).

### Run the industrial_robot_client node (ROS-I client)

The `kinetic-devel` branch provides launch files (within the `staubli_val3_driver`
ROS package). Simply run:

```shell
roslaunch staubli_val3_driver robot_interface_streaming.launch robot_ip:=<Controller IP address>
```

## Bugs, suggestions and feature requests

Please report any bugs you may find, make any suggestions you may have and request
new features you may find useful via GitHub.
