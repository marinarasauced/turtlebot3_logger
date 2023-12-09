# turtlebot3_logger
## Overview
This repo contains a ROS Noetic package for a TurtleBot3 that logs VICON feedback, IMU data, OpenCR Odometry, wheel velocities and torques, and control inputs to several .csv files on node shutdown. The target user for this repo is anyone with access to AVMI resources or anyone with a VICON system.
## Installation requirements
- A TurtleBot3 device - *if you are working on a new device, following [these steps](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) to install ROS Noetic*
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
Once you have the package built, you can launch the node. The callback functions that log the data are designed to work with the [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) and [VICON bridge](https://github.com/ethz-asl/vicon_bridge) packages. However, data is only logged as it is passed through the callback function, so the node is functional with partial feedback from a VICON system and/or onboard sensors. The TurtleBot3 packages should be installed locally on the TurtleBot3 RasPi whereas the VICON bridge package should be installed on your Ubuntu machine.
  
Start a core on the desktop machine, then launch the bringup function in a terminal onboard the TurtleBot3's RasPi.
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
In a new terminal onboard the TurtleBot3's RasPi, run the node. The .csv output files will be saved to the current working directory, so if you wish to store the .csv files in a specific directory, navigate there first before starting the node.
```
rosrun turtlebot3_logger logger
```
A message should display in the terminal indicating the node was successfully launched and that data is being stored. Data will continue to be stored until a flag is published, forcing the node to shutdown. To shutdown the node, publish a Bool to the "/followline/flag" topic. To publish this message, either add a new publisher to another node that is called when specific conditions are met, or manually publish a message from a terminal on the desktop machine.
```
rostopic pub /followline/flag Bool "data: true"
```
After shutting down the node, two messages should be printed in the TurtleBot3 terminal indicating that the logging has commenced and that the logging is terminated.
