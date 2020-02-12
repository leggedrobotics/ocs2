//
// Created by rgrandia on 12.02.20.
//

#include "ocs2_cart_pole_example/ros_comm/CartpoleDummyVisualization.h"

namespace ocs2 {
namespace cartpole {

void CartpoleDummyVisualization::update(const system_observation_t& observation, const primal_solution_t& policy,
                                        const command_data_t& command) {
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] = "slider_to_cart";
  joint_state.name[1] = "cart_to_pole";
  joint_state.position[0] = observation.state()(1);
  joint_state.position[1] = observation.state()(0);
  jointPublisher_.publish(joint_state);
}

void CartpoleDummyVisualization::launchVisualizerNode(ros::NodeHandle& n) {
  jointPublisher_ = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ROS_INFO_STREAM("Waiting for visualization subscriber ...");
  while (ros::ok() && jointPublisher_.getNumSubscribers() == 0) {
    ros::Rate(100).sleep();
  }
  ROS_INFO_STREAM("Visualization subscriber is connected.");
}

}  // namespace cartpole
}  // namespace ocs2
