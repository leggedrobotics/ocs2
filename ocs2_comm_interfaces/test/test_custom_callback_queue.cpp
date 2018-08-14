//
// Created by ruben on 14.08.18.
//

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ros/callback_queue.h>
#include <thread>

// Global queue: will be available as a member variable
ros::CallbackQueue my_queue;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  printf("I heard: [%s]\n", msg->data.c_str());
}

void subscriberWorker()
{
  ros::Rate loop_rate(30);
  while (ros::ok()){
    std::cout << "Check for new messages" << std::endl;
    my_queue.callOne();
    loop_rate.sleep();
  }
}

int main( int argc, char* argv[] ){
  ros::init(argc, argv, "my_node");

  // Publisher
  ros::NodeHandle nh_pub;
  size_t publish_queue_size = 1000;
  ros::Publisher chatter_pub = nh_pub.advertise<std_msgs::String>("chatter", publish_queue_size);

  // Subscriber
  ros::NodeHandle nh_sub;
  nh_sub.setCallbackQueue(&my_queue);

  size_t subscribe_queue_size = 1000;
  ros::Subscriber sub = nh_sub.subscribe("chatter", subscribe_queue_size, chatterCallback);

  auto subscriberThread = std::thread(&subscriberWorker);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    chatter_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  if (subscriberThread.joinable()) { subscriberThread.join(); };

  return 0;
}