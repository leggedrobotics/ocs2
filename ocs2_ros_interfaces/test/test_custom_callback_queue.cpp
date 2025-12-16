/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Global queue: will be available as a member variable
rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;

void chatterCallback(const std_msgs::msg::String& msg) {
  printf("I heard: [%s]\n", msg.data.c_str());
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  // Publisher
  std::shared_ptr<rclcpp::Node> node_pub = rclcpp::Node::make_shared("test_custom_callback_publisher");
  size_t publish_queue_size = 1000;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr chatter_pub = node_pub->create_publisher<std_msgs::msg::String>("chatter", publish_queue_size);

  // Subscriber
  std::shared_ptr<rclcpp::Node> node_sub = rclcpp::Node::make_shared("test_custom_callback_subscriber");
  size_t subscribe_queue_size = 1000;
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = node_sub->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub = node_sub->create_subscription<std_msgs::msg::String>("chatter", subscribe_queue_size, &chatterCallback, sub_opt);

  executor.add_node(node_pub);
  executor.add_node(node_sub);

  rclcpp::Rate loop_rate(10);
  int count = 0;
  while (rclcpp::ok()) {
    std_msgs::msg::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    chatter_pub->publish(msg);
    executor.spin_once();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
