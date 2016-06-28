#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <geometry_msgs/PoseArray.h>

#include <kalman_filter.h>

ros::Publisher filter_output_pub;

ros::Subscriber filter_input_sub;

std::string input_topic;
std::string output_topic;

std_msgs::Float64 filter_output_msg;

KalmanFilter *kf;

double new_state;

void newStateCallback(const std_msgs::Float64 filter_input_msg)
{
  new_state = filter_input_msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_kalman_filter");
  ROS_INFO("Starting a single kalman filter");
  ros::NodeHandle nh("~");

  double P = 1;
  double x_hat = 0;
  double Q = 0.00008;
  double R = 10;


  nh.param<std::string>("input_topic", input_topic, "/teamhku/kalman_filter/dummy_input");
  nh.param<std::string>("output_topic", output_topic, "/teamhku/kalman_filter/dummy_output");
  nh.param<double>("Q", Q, 0.00008);
  nh.param<double>("R", R, 10);
  nh.param<double>("x0", x_hat, 0);

  ROS_INFO("Listening to input topic: %s", input_topic.c_str());
  ROS_INFO("Publishing to output topic: %s", output_topic.c_str());
  ROS_INFO("Filter parameters: ");
  ROS_INFO("Q: %f", Q);
  ROS_INFO("R: %f", R);
  ROS_INFO("x0: %f", x_hat);

  filter_output_pub = nh.advertise<std_msgs::Float64>(output_topic, 10);
  filter_input_sub = nh.subscribe(input_topic, 10, newStateCallback);

  kf = new KalmanFilter(P, Q, R);
 
  kf->init();

  ros::Rate loop_rate(200); 

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_DEBUG_ONCE("Publishing the tag detection data...");

    kf->update(new_state);


    filter_output_msg.data = kf->state();
  
    filter_output_pub.publish(filter_output_msg);     

    loop_rate.sleep();
  }
}
