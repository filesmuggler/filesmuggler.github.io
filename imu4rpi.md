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
git clone -b kinetic-devel https://github.com/ros/pluginlib.git
git clone -b main https://github.com/GAVLab/ros_icm20948
git clone -b kinetic https://github.com/ccny-ros-pkg/imu_tools.git
```
Enter `imu_tools` and remove `rviz_imu_plugin`. We are not needing this one since we have neither rviz nor desktop installed. Also, it causes errors if you do not remove it beforehand.
```shell
git clone -b indigo-devel https://github.com/ros/nodelet_core
git clone -b master https://github.com/orocos/orocos_kinematics_dynamics.git
```
Enter `orocos_kinematics_dynamics` and follow instructions described [here](https://github.com/orocos/orocos_kinematics_dynamics/blob/master/README.md). However, if you are lazy and willing to take a risk of outdated commands feel free to execute lines below.
```shell
sudo apt-get install python-psutil python-future
sudo apt-get install libeigen3-dev libcppunit-dev
git submodule update --init #installing submodule for PyBind11
```
### Installing missing packages for ros_icm20948
Install packages. based on [original source](https://github.com/GAVLab/ros_icm20948/blob/main/README.md)
```shell
sudo apt-get install i2c-tools
sudo apt install wiringpi
sudo apt-get install python3-scipy
```
Add `i2c-devl` to boot with
```shell
sudo nano /etc/modules-load.d/modules.conf
```
You may find `i2c-dev` already in the `modules.conf` if you enabled I2C communication in [previous tutorial](ros4rpi.md)
Install some python packages
```shell
pip3 install ahrs
sudo pip3 install sparkfun-qwiic-icm20948
```
In case you missed one tutorial with installing python3.6 on RPI please go here.



