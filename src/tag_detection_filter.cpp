#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <geometry_msgs/PoseArray.h>

#include <kalman_filter.h>

ros::Publisher tag_x_pub;
ros::Publisher tag_y_pub;
ros::Publisher tag_z_pub;

ros::Subscriber apriltags_pos_sub;

std::string tag_detection_topic;

std_msgs::Float64 filtered_tag_x_msg;
std_msgs::Float64 filtered_tag_y_msg;
std_msgs::Float64 filtered_tag_z_msg;

// double P;
// double P_minus;
// double Q;
// double R;
// double x_hat;
// double x_hat_minus;
// double K;

double z_x;
double z_y;
double z_z;

KalmanFilter *kf_x;
KalmanFilter *kf_y;
KalmanFilter *kf_z;


void apriltagsPositionCallback(const geometry_msgs::PoseArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->poses) == std::end(apriltag_pos_msg->poses))
  {
    return;
  }
  else
  {

    z_x = apriltag_pos_msg->poses[0].position.x;
    z_y = apriltag_pos_msg->poses[0].position.y;
    z_z = apriltag_pos_msg->poses[0].position.z;

  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_detection_filtered_data_publisher");
  ROS_INFO("Starting tag detection filtered data publisher");
  ros::NodeHandle nh;

  nh.param<std::string>("/tag_detection_filter/tag_detection_topic", tag_detection_topic, "/apriltags_ros/tag_detections_pose");

  ROS_INFO("Listening to apriltag detection topic: %s", tag_detection_topic.c_str());

  tag_x_pub = nh.advertise<std_msgs::Float64>("/teamhku/filtered_data/tag_detection_x", 1);
  tag_y_pub = nh.advertise<std_msgs::Float64>("/teamhku/filtered_data/tag_detection_y", 1);
  tag_z_pub = nh.advertise<std_msgs::Float64>("/teamhku/filtered_data/tag_detection_z", 1);

  apriltags_pos_sub = nh.subscribe(tag_detection_topic, 1000, apriltagsPositionCallback);

  double P = 1;
  double x_hat = 0;
  double Q = 0.00008;
  double R = 10;

  kf_x = new KalmanFilter(P, Q, R);
  kf_y = new KalmanFilter(P, Q, R);
  kf_z = new KalmanFilter(P, Q, R);

  kf_x->init();
  kf_y->init();
  kf_z->init();

  ros::Rate loop_rate(200); 

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_DEBUG_ONCE("Publishing the tag detection data...");

    // x_hat_minus = x_hat;
    // P_minus = P + Q;
    // K = P_minus / (P_minus + R);
    // x_hat = x_hat_minus + K * (z_x - x_hat_minus);
    // P = (1 - K) * P_minus;
    // filtered_tag_x_msg.data = x_hat;

    kf_x->update(z_x);
    kf_y->update(z_y);
    kf_z->update(z_z);

    filtered_tag_x_msg.data = kf_x->state();
    filtered_tag_y_msg.data = kf_y->state();
    filtered_tag_z_msg.data = kf_z->state();

    tag_x_pub.publish(filtered_tag_x_msg);     
    tag_y_pub.publish(filtered_tag_y_msg);
    tag_z_pub.publish(filtered_tag_z_msg);

    loop_rate.sleep();
  }
}
