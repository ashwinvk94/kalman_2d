# 2D Kalman Filter
This repository contains a ROS C++ implementation of 2d kalman filter. The states being estimated are position_x, position_y, velocity_x and velocity_y and it assumes a constant velocity model for prediction.
The measurements are position_x and position_y.

This filter can be used to with any 2 dimensional stream of data.

## Inputs and Outputs

The measurements are received through a ROS topic and the filtered state is published to the ROS topic.
The data to the filter can be sent asynchronously and the filter runs at the frequency at which measurement data is received.

The filtered data is time stamped with the same header as the measurement data. 

This code is based on kalman filter equations found at : https://en.wikipedia.org/wiki/Kalman_filter

The ros docker interface is derived from : https://github.com/tkunic/ros-docker-simple


## Dependencies
Run the following commands:
`sudo apt install libeigen3-dev`

## Instructions
clone this repository into your ros workspace and build using `catkin_make`
run the filter and an example measurement publisher using `roslaunch kalman_2d kalman.launch` 

The ROS_INFO_STREAM outputs should show:
* groundtruth and corresponding timestamp
* measurements + states and corresponding timestamp

You can also visualize the outputs using [rqt_graph](http://wiki.ros.org/rqt_graph)

![alt text](https://github.com/ashwinvk94/kalman_2d/blob/main/filtered.png?raw=true)

## Instructions
![alt text](https://github.com/ashwinvk94/kalman_2d/blob/main/kalman_equations.png?raw=true)
