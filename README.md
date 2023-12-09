# turtlebot3_logger
## Overview
This repo contains a ROS Noetic package for a TurtleBot3 that logs VICON feedback, IMU data, OpenCR Odometry, wheel velocities and torques, and control inputs to several .csv files on node shutdown. The target user for this repo is anyone with access to AVMI resources or anyone with a VICON system.
## Installation requirements
-A TurtleBot3 device - *if you are working on a new device, following [these steps](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to install ROS Noetic*
- Access to the AVMI local WiFi network
- An Ubuntu 20.04 machine

## Installation instructions
From your remote Ubuntu device, log into the TurtleBot3 using SSH.
'''
ssh ubuntu@{IP_OF_TURTLEBOT3}
'''


