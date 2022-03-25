# How to install ROS on RPI Zero and not to loose your mind

## Prerequisites
### Download image
Download and flash [Raspbian Buster Lite](https://downloads.raspberrypi.org/raspbian_lite/images/raspbian_lite-2020-02-14/) onto 16GB SD Card. At first boot be sure to enable SSH and add your WiFi network credentials.

### Setup Repositories
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install ca-certificates ssl-cert
```
### Add Google DNS
Open:
```
sudo nano /etc/resolv.conf
```
Paste `nameserver 8.8.8.8` and save, reboot.
### Install keys
According to [this source](https://answers.ros.org/question/329434/installing-ros-kinetic-on-the-raspberry-pi-no_pubkey-f42ed6fbab17c654/) execute the key install below.
```
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
Debian Buster is not exactly the newest so you have to accept changes.
```
sudo apt update --allow-releaseinfo-change
sudo apt-get update
sudo apt-get upgrade
```
### Install Bootstrap Dependencies
```
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```
### Init rosdep
```
sudo rosdep init
rosdep update
```
## Installation

### Creating catkin workspace
Create workspace in the home directory
```
mkdir -p ~/ros_catkin_ws/src/
cd ~/ros_catkin_ws/
```
### Core packages
Fetch core packages using wstool command.
```
rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
wstool init src melodic-ros_comm-wet.rosinstall
```
### Resolve Dependencies
Run rosdep 







