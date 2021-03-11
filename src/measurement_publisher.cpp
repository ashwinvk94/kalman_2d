#include "measurement_publisher.h"

MeasurementPublisher::MeasurementPublisher(ros::NodeHandle& n, bool test_flag) : nh_(n)
{
  // Initialize publishers
  pub_measurement_ = n.advertise<nav_msgs::Odometry>("/kalman/measurement", 10);
  
  // create time axis
  int n_samples = 2000;
  
  Eigen::ArrayXf x_ground_truth, y_ground_truth;

  Eigen::ArrayXf x_measured = Eigen::ArrayXf::LinSpaced(n_samples, 0, 100);
  Eigen::ArrayXf y_measured = Eigen::ArrayXf::LinSpaced(n_samples, 0, 100);
  Eigen::ArrayXf x_noise = Eigen::ArrayXf::Random(n_samples);
  Eigen::ArrayXf y_noise = Eigen::ArrayXf::Random(n_samples);
  x_ground_truth = x_measured;
  y_ground_truth = y_measured;
  x_measured = x_measured+x_noise;
  y_measured = y_measured+y_noise;

  publishing_rate = 10; //publishing rate is 10Hz
  ros::Rate rate(publishing_rate);

  unsigned int time = 0;

  nav_msgs::Odometry measurement_msg_;
  double noise = 0.5;

  if(!test_flag)
  {
    while(ros::ok() && time<n_samples){

      measurement_msg_.header.stamp = ros::Time::now();
      measurement_msg_.pose.pose.position.x = x_measured(time);
      measurement_msg_.pose.pose.position.y = y_measured(time);
      measurement_msg_.pose.covariance[0] = noise;
      measurement_msg_.pose.covariance[1] = 0;
      measurement_msg_.pose.covariance[6] = 0;
      measurement_msg_.pose.covariance[7] = noise;

      pub_measurement_.publish(measurement_msg_);

      ROS_INFO_STREAM("timestamp : "<<measurement_msg_.header.stamp);
      ROS_INFO_STREAM("ground_truth x : "<<x_ground_truth(time));
      ROS_INFO_STREAM("ground_truth y : "<<y_ground_truth(time));

      time++;
      ros::spinOnce();
      rate.sleep();
    }    
  }
}

MeasurementPublisher::~MeasurementPublisher()
{
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "measurement_publisher");
  ros::NodeHandle n;
  ROS_INFO("Measurement Publisher node initiated");
  MeasurementPublisher measurement_publisher(n);
  ros::shutdown();
  return 0;
}