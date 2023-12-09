# turtlebot3_logger
## Overview
This repo contains a ROS Noetic package for a TurtleBot3 that logs VICON feedback, IMU data, OpenCR Odometry, wheel velocities and torques, and control inputs to several .csv files on node shutdown. The target user for this repo is anyone with access to AVMI resources or anyone with a VICON system.
## Installation requirements
-A TurtleBot3 device - *if you are working on a new device, following [these steps](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to install ROS Noetic*
- Access to the AVMI local WiFi network
- An Ubuntu 20.04 machine

## Installation instructions
From your remote Ubuntu device, log into the TurtleBot3 using SSH, and enter the device password.
```
ssh ubuntu@{IP_OF_TURTLEBOT3}
```
Navigate to the source folder of your catkin workspace and clone the repository. This step requires the TurtleBot3 to have an active internet connection.
```
cd ~/catkin_ws/src && git clone https://github.com/marinarasauced/turtlebot3_logger.git
```
Return to the catkin workspace and compile the package.
```
cd ~/catkin_ws && catkin_make
```
## Usage instructions
Once you have the package built, you can launch the node. The callback functions that log the data are designed to work with the [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [VICON bridge](https://github.com/ethz-asl/vicon_bridge) packages. However, data is only logged as it is passed through the callback function, so the node is functional with partial feedback from a VICON system and/or onboard sensors. The TurtleBot3 packages should be installed locally on the TurtleBot3 RasPi while the VICON bridge package should be installed on a ground station (desktop).
