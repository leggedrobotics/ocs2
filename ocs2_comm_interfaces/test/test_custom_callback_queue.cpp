//
// Created by ruben on 14.08.18.
//

#include <ros/ros.h>
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main( int argc, char* argv[] ){
  ros::init(argc, argv, "my_node");
  ros::NodeHandle nh;

  size_t publish_queue_size = 1000;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", publish_queue_size);

  size_t subscribe_queue_size = 1000;
  ros::Subscriber sub = nh.subscribe("chatter", subscribe_queue_size, chatterCallback);

}