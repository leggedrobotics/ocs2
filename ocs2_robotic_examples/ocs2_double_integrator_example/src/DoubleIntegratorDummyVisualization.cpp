//
// Created by rgrandia on 12.02.20.
//

#include "ocs2_double_integrator_example/ros_comm/DoubleIntegratorDummyVisualization.h"

namespace ocs2 {
namespace double_integrator {

void DoubleIntegratorDummyVisualization::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  jointPublisher_ = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 1);
}

void DoubleIntegratorDummyVisualization::update(const system_observation_t& observation, const primal_solution_t& policy,
                                                const command_data_t& command) {
  const auto& costDesiredTrajectories = command.mpcCostDesiredTrajectories_;
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] = "slider_to_cart";
  joint_state.position[0] = observation.state()(0);
  joint_state.name[1] = "slider_to_target";
  joint_state.position[1] = costDesiredTrajectories.desiredStateTrajectory()[0](0);

  jointPublisher_.publish(joint_state);
}

}  // namespace double_integrator
}  // namespace ocs2
