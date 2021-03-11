#include "kalman.h"

Kalman::Kalman(ros::NodeHandle& n, bool test_flag) : nh_(n)
{
  // Flag to check for first measurement
  meas_received_ = false;

  // Initialize publishers
  pub_state_ = n.advertise<nav_msgs::Odometry>("/kalman/state", 10);

  pub_measurement_ = n.advertise<nav_msgs::Odometry>("/kalman/measurement", 10);

  sub_measurement_ = n.subscribe<nav_msgs::Odometry>("/kalman/measurement",1, \
                                                &Kalman::measurementCallback, this);

  // initialize kalman filter values
  initKalman();
}

Kalman::~Kalman()
{
}

void Kalman::initKalman(){
  state_<< 0,0,0,0;

  // Uncertaininty matrix initialization
  P_.setIdentity();
  P_ = P_*1000;

  // Initialize measurement uncertainity
  R_ << 0.5, 0,
        0, 0.5;

  // Initialize state transformation matrix
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  // Initialize observation transformation matrix
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

}

void Kalman::runKalman(Eigen::Matrix<double,4,1> &state, Eigen::Matrix<double,4,4> &P, \
			Eigen::Matrix<double,2,1> meas, Eigen::Matrix<double,2,2> R){

  // Calculate innovation and innovation covariance
  Eigen::Matrix<double,2,1> innovation = meas - H_*state;
  Eigen::Matrix<double,2,2> innovation_cov = H_ * P * H_.transpose() + R;

  // Calculate kalman gain
  Eigen::Matrix<double,4,2> K = P * H_.transpose() * innovation_cov.inverse();

  // Update state using kalman gain
  state = state + K*innovation;
  // Identity with the same size as the state transition matrix
  Eigen::Matrix<double,4,4> I;
  I.setIdentity();
  // Update state covariance matrix
  P = (I - K*H_)*P;

}

void Kalman::predictKalman(){
  // Prediction of next future state
  state_ = F_*state_;
  // state = F_*state + input; // TO DO : add input
  P_ = F_*P_*F_.transpose();
  // P = F_*P*F_.transpose() + Q; // TO DO : add process noise
}

void Kalman::measurementCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(meas_received_){
    // we can only predict if kalamn filter has run atleast once
    double dt = (msg->header.stamp-prev_timestamp_).toSec();
    // diffrences between measurements
    F_(0,2) = dt;
    F_(1,3) = dt;

    // predict the current state using previous estimates
    predictKalman();
  }
  // read measured values
  meas_ << msg->pose.pose.position.x, msg->pose.pose.position.y;
  
  // covariance matrix for nav_msgs::Odometry is expanded in the row major form
  R_ << msg->pose.covariance[0], msg->pose.covariance[1],
        msg->pose.covariance[6], msg->pose.covariance[7];
  
  // calling kalman filter class
  runKalman(state_, P_, meas_, R_);

  ROS_INFO_STREAM("timestamp : "<<msg->header.stamp);
  ROS_INFO_STREAM("measured x : "<<meas_(0));
  ROS_INFO_STREAM("measured y : "<<meas_(0));
  ROS_INFO_STREAM("state x : "<<state_(0));
  ROS_INFO_STREAM("state y : "<<state_(1));
  ROS_INFO_STREAM("state vx : "<<state_(2));
  ROS_INFO_STREAM("state vy : "<<state_(3));

  
  updateStateMsg(msg->header);

  pub_state_.publish(state_msg_);
  ROS_INFO_STREAM("\n");

  prev_timestamp_ = msg->header.stamp;
  meas_received_ = true;
}

void Kalman::updateStateMsg(std_msgs::Header header){
  // timestamp of measurement and filtered data is the same
  state_msg_.header = header;
  state_msg_.pose.pose.position.x = state_(0);
  state_msg_.pose.pose.position.y = state_(1);
  state_msg_.twist.twist.linear.x = state_(2);
  state_msg_.twist.twist.linear.y = state_(3);
  state_msg_.pose.covariance[0] = P_(0,0);
  state_msg_.pose.covariance[1] = P_(0,1);
  state_msg_.pose.covariance[6] = P_(1,0);
  state_msg_.pose.covariance[7] = P_(1,1);
  state_msg_.twist.covariance[0] = P_(2,2);
  state_msg_.twist.covariance[1] = P_(2,3);
  state_msg_.twist.covariance[6] = P_(3,2);
  state_msg_.twist.covariance[7] = P_(3,3);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "kalman_filter");
  ros::NodeHandle n;
  Kalman kalman_filter(n);
  ROS_INFO("Kalman filter node initiated");
  ros::spin();
  return 0;
}