/**
 * @file    measurement_publisher.h
 * @author  Ashwin (ashwinvk94@gmail.com)
 * @brief	This class publishes raw position data with uniform
 * 			distributed noise
 * 			
 * @version 1.0
 * @date    10 March 2021
 * 
 */
#ifndef MEASUREMENT_PUBLISHER_H
#define MEASUREMENT_PUBLISHER_H
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

class MeasurementPublisher {
private:
	// Ros node handle
	ros::NodeHandle nh_;

	//main while loop rate
	int publishing_rate;
	
	// Publishers
	// Publish current state of the drone
	ros::Publisher pub_measurement_;

	// ROS state message
	nav_msgs::Odometry measurement_msg_;

public:
	/**
	 * @brief Construct a new MeasurementPublisher Filter object
	 * 
	 * @param n 
	 */
	MeasurementPublisher(ros::NodeHandle& n, bool test_flag = false);

	/**
	 * @brief Destroy the MeasurementPublisher Filter object
	 * 
	 */
	~MeasurementPublisher();
};
#endif // MEASUREMENT_PUBLISHER_H