/**
 * @file    kalman.h
 * @author  Ashwin (ashwinvk94@gmail.com)
 * @brief	This class implements a 2d kalman filter
 * 			An explanation and equations in the kalamn filter can be found at :
 * 			https://en.wikipedia.org/wiki/Kalman_filter
 * @version 1.0
 * @date    8 March 2021
 * 
 */
#ifndef KALMAN_H
#define KALMAN_H
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

class Kalman {
private:
	// Ros node handle
	ros::NodeHandle nh_;
	
	// Publishers
	// Publish current state of the drone
	ros::Publisher pub_state_, pub_measurement_;

	// Subscriber
	ros::Subscriber sub_measurement_;

	// ROS state message
	nav_msgs::Odometry state_msg_;

	// previous timestamp header
	ros::Time prev_timestamp_;

	// Flag to check for first measurement 
	bool meas_received_;

	// Kalman filter Initialization
	// state consists of
	// pos_x
	// pos_y
	// vel_x
	// vel_y
	Eigen::Matrix<double,4,1> state_;

	// State Uncertaininty matrix 
  	Eigen::Matrix<double,4,4> P_;

	// Measurements 
  	Eigen::Matrix<double,2,1> meas_;

	// Measurement Noise 
	Eigen::Matrix<double,2,2> R_;

	// State Transformation Matrix
	Eigen::Matrix<double,4,4> F_;

	// Observation Transformation Matrix
	Eigen::Matrix<double,2,4> H_;

public:
	/**
	 * @brief Construct a new Kalman Filter object
	 * 
	 * @param n 
	 */
	Kalman(ros::NodeHandle& n, bool test_flag = false);

	/**
	 * @brief Destroy the Kalman Filter object
	 * 
	 */
	~Kalman();
	
	/**
	 * @brief Initialize all kalam filter variables
	 * 
	 */
	void initKalman();

	/**
	 * @brief Initialize all kalam filter variables
	 * 
	 */
	void runKalman(Eigen::Matrix<double,4,1> &state, Eigen::Matrix<double,4,4> &P, \
			Eigen::Matrix<double,2,1> meas, Eigen::Matrix<double,2,2> R);

	/**
	 * @brief Predict the next iterati
	 * 
	 */
	void predictKalman();

	/**
	 * @brief Callback to read measurement values 
	 * 
	 * @param msg Measurement data with timestamp
	 */
	void measurementCallback(const nav_msgs::Odometry::ConstPtr& msg);

	/**
	 * @brief write the current state to the state ros message 
	 * 
	 */
	void updateStateMsg(std_msgs::Header header);
};
#endif // KALMAN_H