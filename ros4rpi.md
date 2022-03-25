# How to install ROS on RPI Zero and not to loose your mind

## Motivation
I was faced multiple times with a challenge to run sensors on RPI Zero W under ROS 1, which can be a tedious in preparation. Despite long presence of both the budget RPI board and ROS at the market, it was not clearly documented how to properly aggregate them. The latest challenge was to put IMU sensor altogether with RPI Zero W board. Here is how it's done.

## Prerequisites
### Download image
Download and flash [Raspbian Buster Lite](https://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2020-02-14/) onto 16GB SD Card. At first boot be sure to enable SSH and add your WiFi network credentials.

### Setup Repositories
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install ca-certificates ssl-cert
```
### Add Google DNS
Open:
```shell
sudo nano /etc/resolv.conf
```
Paste `nameserver 8.8.8.8` and save, reboot.
### Install keys
According to [this source](https://answers.ros.org/question/329434/installing-ros-kinetic-on-the-raspberry-pi-no_pubkey-f42ed6fbab17c654/) execute the key install below.
```shell
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
Debian Buster is not exactly the newest so you have to accept changes.
```shell
sudo apt update --allow-releaseinfo-change
sudo apt-get update
sudo apt-get upgrade
```
If above commands gives you error like:
```shell
E: Release file for http://packages.ros.org/ros/ubuntu/dists/buster/InRelease is not valid yet (invalid for another 769d 16h 34min 23s). Updates for this repository will not be applied.
```
please be sure to update your date manually with command:
```shell
sudo date -s '25 Mar 2022 15:37'
```
but insert your date and time GMT. Then retry update & upgrade.

### Install Bootstrap Dependencies
```shell
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```
### Init rosdep
```shell
sudo rosdep init
rosdep update
```
## Installation

### Creating catkin workspace
Create workspace in the home directory
```shell
mkdir -p ~/ros_catkin_ws/src/
cd ~/ros_catkin_ws/
```
### Core packages
Fetch core packages using wstool command.
```shell
rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
wstool init src melodic-ros_comm-wet.rosinstall
```
### Resolve Dependencies
Run rosdep. It will recursively install packages.

```shell
cd ~/ros_catkin_ws
rosdep install -y --from-paths src --ignore-src --rosdistro melodic -r --os=debian:buster
```
___
#### Comment
_The --from-paths option indicates we want to install the dependencies for an entire directory of packages, in this case src. The --ignore-src option indicates to rosdep that it shouldn't try to install any ROS packages in the src folder from the package manager, we don't need it to since we are building them ourselves. The --rosdistro option is required because we don't have a ROS environment setup yet, so we have to indicate to rosdep what version of ROS we are building for. Finally, the -y option indicates to rosdep that we don't want to be bothered by too many prompts from the package manager._ 
[Source](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi)
___

### Building catkin workspace
When you complete steps above, run building. modifier `-j1` ensures that you will not run into memory problems (more or less).
```shell
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j1
```
It takes some time since RPI Zero CPU is rather slow.
Source the ROS environment every time you open a bash session.
```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

### Testing ROS
Run. If everything is correctly build you should be able to view a standard ROS output.
```shell
source /opt/ros/melodic/setup.bash
roscore
```



