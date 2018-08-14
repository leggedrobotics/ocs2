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

  size_t message_queue_size = 1000;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", message_queue_size);

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::Subscriber sub = n.subs


}