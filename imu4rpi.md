# How to run IMU under ROS Melodic on RPI Zero and survive

## Motivation
RPI Zero W is a low cost computer, succeded by RPI Zero 2 W, still popular in many applications, especially right now
when shortage hit the electronic market all around the world. It's also much slower than his younger brother therefore
makes it more challenging to use. Today you are going to learn how to survive installing all packages for IMU.

## Prerequisites
You should have RPI Zero W prepared from the [previous tutorial](ros4rpi.md), where you install ROS Melodic on RPI Zero W with Rasbian Buster.

## Installation
### Workspace
Create separate workspace in your home directory 
```shell
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
```
Git clone required packages
```shell
git clone -b indigo-devel https://github.com/ros/actionlib.git
git clone https://github.com/ros/angles
git clone -b kinetic-devel https://github.com/ros/bond_core.git
git clone -b indigo-devel https://github.com/ros/class_loader
git clone -b noetic-devel https://github.com/ros/common_msgs
git clone -b master https://github.com/ros/dynamic_reconfigure.git
git clone -b indigo-devel https://github.com/ros/geometry.git
git clone -b noetic-devel https://github.com/ros/geometry2

```
