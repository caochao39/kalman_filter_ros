#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <cmath>
#include <geometry_msgs/PoseArray.h>

ros::Publisher tag_x_pub;
ros::Publisher tag_y_pub;
ros::Publisher tag_z_pub;

ros::Subscriber apriltags_pos_sub;


std::string tag_detection_topic;

double tag_x;
double tag_y;
double tag_z;

std_msgs::Float64 tag_x_msg;
std_msgs::Float64 tag_y_msg;
std_msgs::Float64 tag_z_msg;


void apriltagsPositionCallback(const geometry_msgs::PoseArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->poses) == std::end(apriltag_pos_msg->poses))
  {
    return;
  }
  else
  {
    tag_x = apriltag_pos_msg->poses[0].position.x;
    tag_y = -apriltag_pos_msg->poses[0].position.y;
    tag_z = apriltag_pos_msg->poses[0].position.z;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tag_detection_raw_data_publisher");
  ROS_INFO("Starting tag detection raw data publisher");
  ros::NodeHandle nh;

  nh.param<std::string>("/tag_detection_raw_data_publisher/tag_detection_topic", tag_detection_topic, "/apriltags_ros/tag_detections_pose");

  ROS_INFO("Listening to apriltag detection topic: %s", tag_detection_topic.c_str());

  tag_x_pub = nh.advertise<std_msgs::Float64>("/teamhku/raw_data/tag_detection_x", 1);
  tag_y_pub = nh.advertise<std_msgs::Float64>("/teamhku/raw_data/tag_detection_y", 1);
  tag_z_pub = nh.advertise<std_msgs::Float64>("/teamhku/raw_data/tag_detection_z", 1);


  apriltags_pos_sub = nh.subscribe(tag_detection_topic, 1000, apriltagsPositionCallback);

  ros::Rate loop_rate(200); 

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_DEBUG_ONCE("Publishing the tag detection data...");
    tag_x_msg.data = tag_x;
    tag_y_msg.data = tag_y;
    tag_z_msg.data = tag_z;

    tag_x_pub.publish(tag_x_msg);     
    tag_y_pub.publish(tag_y_msg);
    tag_z_pub.publish(tag_z_msg);

    loop_rate.sleep();
  }
}
