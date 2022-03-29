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
Install required packages.
```shell
sudo apt-get install python-psutil python-future
sudo apt-get install libeigen3-dev libcppunit-dev
```
Install the submodule.
```shell
git submodule update --init #installing submodule for PyBind11
```
### Installing missing packages for ros_icm20948
Install packages. based on [original source](https://github.com/GAVLab/ros_icm20948/blob/main/README.md)
```shell
sudo apt-get install i2c-tools wiringpi python3-scipy python-pip
```
Install some python3.6 packages
```shell
python3.6 -m pip install Cython numpy ahrs sparkfun-qwiic-icm20948 pyyaml PyYAML
python3.6 -m pip install adafruit-blinka==6.10.0 adafruit-circuitpython-icm20x==2.0.7 Adafruit-PlatformDetect=3.6.0
python3.6 -m pip install adafruit-circuitpython-busdevice==5.0.6 adafruit-python-shell
```
Install some python2 packages
```shell
pip install pyyaml PyYAML
```
These are the versions of the packages that are proven to work for python3.6 on RPI with melodic. However
In case you missed one tutorial with installing python3.6 on RPI please go here.
### Add I2C permission
Add `i2c-devl` to boot with
```shell
sudo nano /etc/modules-load.d/modules.conf
```
You may find `i2c-dev` already in the `modules.conf` if you enabled I2C communication in [previous tutorial](ros4rpi.md)

Before you build anything be sure to have `bullet` library installed.
```shell
sudo apt-get install libbullet-dev
```
### Building time!
Build the workspace with
```shell
catkin_make_isolated
```
It will take a while so here is something to keep you happy.
<iframe width="560" height="315" src="https://www.youtube.com/embed/eY52Zsg-KVI" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

After the workspace is built, go to 
```shell
sudo nano /etc/dhcpcd.conf
```
and paste at the end configuration coresponding to you LAN setup. In my case it looks like this.
It sets static IP address, gateway and DNS for the RPI on the wifi interface.
```
interface wlan0
static ip_address=192.168.1.101/24
static routers=192.168.1.1
static domain_name_servers=192.168.1.1
```
Make sure that in `/etc/wpa_supplicant/wpa_supplicant.conf` you have your network SSID with password key added.
now you can reboot, log in and continue in the next section.
## ROS nodes
Enter `~/catkin_ws/src/` and create new package with `catkin_tools`
```shell
catkin_create_pkg imu_rpi std_msgs rospy roscpp
```
Go to `src` folder of the newly created package and create test script `nano imu_test.py`.
```python
from __future__ import print_function
import qwiic_icm20948
import time
import sys

def runExample():

	print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
	IMU = qwiic_icm20948.QwiicIcm20948()

	if IMU.connected == False:
		print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return

	IMU.begin()

	while True:
		if IMU.dataReady():
			IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
			print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (IMU.axRaw,IMU.ayRaw,IMU.azRaw))
			print("Gyro X:%.2f, Y: %.2f, Z: %.2f rads/s" % (IMU.gxRaw,IMU.gyRaw,IMU.gzRaw))
			print("Magnetometer X:%.2f, Y: %.2f, Z: %.2f uT" % (IMU.mxRaw,IMU.myRaw,IMU.mzRaw))
			print("")
			time.sleep(0.03)
		else:
			print("Waiting for data")
			time.sleep(0.5)

if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example 1")
		sys.exit(0)
```
If above code works you are good to go and create ROS node `nano imu_talker.py`.
Then paste the contents of the file below. 
```python
#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import MagneticField,Imu
from std_msgs.msg import Float64
import time
import board
import busio
import os
import numpy as np
from adafruit_icm20x import ICM20948,AccelRange,GyroRange

def imu_node():
        print("dupa")
        raw_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
        mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
        rospy.init_node('icm20948')
        rate = rospy.Rate(100)
        rospy.loginfo(rospy.get_caller_id() + "  icm20948 node launched.")

        i2c = busio.I2C(board.SCL, board.SDA)
        icm = ICM20948(i2c)
        icm.accelerometer_range = AccelRange.RANGE_4G # Options: RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G
        rospy.loginfo(rospy.get_caller_id() + " Initializing IMU module...")
        time.sleep(5)
        rospy.loginfo(rospy.get_caller_id() + " IMU module set")
        if icm.gyro_range == 0:
                gyro_range = 250
        elif icm.gyro_range == 1:
                gyro_range = 500
        elif icm.gyro_range == 2:
                gyro_range = 1000
        elif icm.gyro_range == 3:
                gyro_range = 2000
        else:
                gyro_range = i
        time.sleep(1)

        frequency = 100  # frequency in Hertz

        rospy.loginfo(rospy.get_caller_id() + " starting to publish")
        while not rospy.is_shutdown():
                acc_data = icm.acceleration # linear acceleration (m/s^2) x,y,z
                gyr_data = icm.gyro # angular velocity (rad/s) x,y,z
                mag_data = tuple(i for i in icm.magnetic) # magnetic field (uT) x,y,z
                rospy.loginfo(rospy.get_caller_id() + " publishing")
                raw_msg = Imu()
                raw_msg.header.frame_id = "imu"
                raw_msg.header.stamp = rospy.Time.now()
                raw_msg.orientation.w = 0
                raw_msg.orientation.x = 0
                raw_msg.orientation.y = 0
                raw_msg.orientation.z = 0
                raw_msg.linear_acceleration.x = acc_data[0]
                raw_msg.linear_acceleration.y = acc_data[1]
                raw_msg.linear_acceleration.z = acc_data[2]
                raw_msg.angular_velocity.x = gyr_data[0]
                raw_msg.angular_velocity.y = gyr_data[1]
                raw_msg.angular_velocity.z = gyr_data[2]
                raw_msg.orientation_covariance[0] = -1
                raw_msg.linear_acceleration_covariance[0] = -1
                raw_msg.angular_velocity_covariance[0] = -1
                raw_pub.publish(raw_msg)
                mag_msg = MagneticField()
                mag_msg.header.stamp = rospy.Time.now()
                mag_msg.magnetic_field.x = mag_data[0]
                mag_msg.magnetic_field.y = mag_data[1]
                mag_msg.magnetic_field.z = mag_data[2]
                mag_msg.magnetic_field_covariance[0] = -1
                mag_pub.publish(mag_msg)
                rate.sleep()

        rospy.loginfo(rospy.get_caller_id() + "  icm20948 node finished")

if __name__ == '__main__':
        try:
                imu_node()
        except rospy.ROSInterruptException:
                rospy.loginfo(rospy.get_caller_id() + "  icm20948 node exited with exception.")
```
It lets you convert raw readings of the IMU module into a ROS topic with `Imu` message type. Go back to root folder of the package and create launch folder `mkdir launch`. Get inside and `nano icm20948.launch`, then paste.
```xml
<launch>
    <node name="imu_rpi_node" pkg="imu_rpi" type="imu_talker.py" respawn="true" respawn_delay="2" >
    </node>
</launch>
```




