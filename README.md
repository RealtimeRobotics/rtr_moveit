# rtr_moveit

Description: A MoveIt! planning plugin for [Realtime Robotics](http://rtr.ai/)'s RapidPlan.

<img src="https://picknik.ai/images/logo.jpg" width="100">

Developed by Henning Kayser at [PickNik Consulting](http://picknik.ai/)

Travis:
[![Build Status](https://travis-ci.com/PickNikRobotics/rtr_moveit.svg?token=o9hPQnr2kShM9ckDs6J8&branch=master)](https://travis-ci.com/PickNikRobotics/rtr_moveit)

### Plugin Structure

The plugin consists of the three classes RTRPlannerManager, RTRPlanningContext and RTRPlannerInterface.
While the first two implement the plugin structure of Moveit!, the RTRPlannerInterface provides and interface for sending motion planning requests to RapidPlan.

RapidPlan's HardwareInterface and PathPlanner need to be initialized with the same roadmaps at each query.
At the moment, it is not possible to retrieve the currently loaded roadmap of the PathPlanner and the stored roadmaps on the Hardware.
The RTRPlannerInterface keeps track of loaded and stored roadmaps and handles write access.
To prevent race conditions, calls of the PathPlanner and HardwareInterface are synchronized with a mutex lock.
The interface hides all of this and only provides functions for initialization, availability and planning attempts.

## Install

### Build from Source

These instructions assume you are running on Ubuntu 16.04:

1. [Install ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and the following build tools.

        sudo apt-get install python-wstool python-catkin-tools

1. Install RapidPlan dependencies:

> Note: RapidPlan is not released yet. All dependencies are installed from [this](https://drive.google.com/uc?id=1X6suaBlgbfmOzGF2sXNSuyOLPkid484R) encrypted binary shell script.

        sudo apt-get install tar mcrypt python-pip
        pip install --user gdown
        gdown https://drive.google.com/uc?id=1X6suaBlgbfmOzGF2sXNSuyOLPkid484R
        cat rtr_0.1.2-133.tar.gz.crypt | crypt "super secret picknik pass" -d > rtr_0.1.2-133.tar.gz
        tar xfz rtr_0.1.2-133.tar.gz
        sudo bash rtr_0.1.2-133/rtr_0.1.2-133.deb.run
        sudo cp rtr_0.1.2-133/RapidPlanDataTypes.hpp /opt/ros/kinetic/include/rtr-api/

> Note: you may need to uninstall older versions of RapidPlan, which you can do via ``sudo apt remove rtr* ros-kinetic-rtr*``

1. Re-use or create a catkin workspace:

        export CATKIN_WS=~/ws_catkin/
        mkdir -p $CATKIN_WS/src
        cd $CATKIN_WS/src

1. Download the required repositories and install any dependencies:

        git clone git@github.com:RealtimeRobotics/rtr_moveit.git
        wstool init .
        wstool merge rtr_moveit/rtr_moveit.rosinstall
        wstool update
        rosdep install --from-paths . --ignore-src --rosdistro kinetic

1. Configure and build the workspace:

        cd $CATKIN_WS
        catkin config --extend /opt/ros/kinetic --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

1. Source the workspace.

        source devel/setup.bash

## Run

See the [rtr_moveit tutorial](https://github.com/RealtimeRobotics/rtr_moveit/blob/master/rtr_moveit_tutorial/rtr_moveit_tutorial.rst) on how to configure and run this plugin.

## Developers: Quick update code repositories

To make sure you have the latest repos:

    cd $CATKIN_WS/src/rtr_moveit
    git checkout master
    git pull origin master
    cd ..
    wstool merge rtr_moveit/rtr_moveit.rosinstall
    wstool update
    rosdep install --from-paths . --ignore-src --rosdistro kinetic

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin lint -W2 --rosdistro kinetic

Use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/) to run tests.

    catkin run_tests --no-deps --this -i
