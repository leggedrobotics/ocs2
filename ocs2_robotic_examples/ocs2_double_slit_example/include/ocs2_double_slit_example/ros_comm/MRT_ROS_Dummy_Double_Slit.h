#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <ocs2_comm_interfaces/test/MRT_ROS_Dummy_Loop.h>

#include "ocs2_double_slit_example/definitions.h"

namespace ocs2 {
namespace double_slit {

class MrtRosDummyDoubleSlit final : public MRT_ROS_Dummy_Loop<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = MRT_ROS_Dummy_Loop<double_slit::STATE_DIM_, double_slit::INPUT_DIM_>;

  /**
   * Constructor.
   *
   * @param [in] mrtPtr
   * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz. This should always set to a positive number.
   * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz. If set to a positive number, MPC loop
   * will be simulated to run by this frequency. Note that this might not be the MPC's realtime frequency.
   */
  MrtRosDummyDoubleSlit(const mrt_ptr_t& mrtPtr, const scalar_t& mrtDesiredFrequency, const scalar_t& mpcDesiredFrequency,
                        controlled_system_base_t* system = nullptr, Rollout_Settings rolloutSettings = Rollout_Settings())
      : BASE(mrtPtr, mrtDesiredFrequency, mpcDesiredFrequency, system, rolloutSettings) {}

  /**
   * Destructor.
   */
  ~MrtRosDummyDoubleSlit() override = default;

 protected:
  /**
   * Launches the visualization node
   *
   * @param [in] argc: command line number of inputs.
   * @param [in] argv: command line inputs' value.
   */
  void launchVisualizerNode(int argc, char* argv[]) override {
    ros::init(argc, argv, "double_slit_visualization_node");
    ros::NodeHandle n;
    jointPublisher_ = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ROS_INFO_STREAM("Waiting for visualization subscriber ...");
    while (ros::ok() && jointPublisher_.getNumSubscribers() == 0) {
      ros::Rate(100).sleep();
    }
    ROS_INFO_STREAM("Visualization subscriber is connected.");
  }

  void publishVisualizer(const system_observation_t& observation, const commandData_t& command, const policyData_t& policy) override {
    const auto& costDesiredTrajectories = command.mpcCostDesiredTrajectories_;
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.name[0] = "xPos";
    joint_state.position[0] = observation.state()(0);
    joint_state.name[1] = "target";
    joint_state.position[1] = costDesiredTrajectories.desiredStateTrajectory()[0](0);

    jointPublisher_.publish(joint_state);
  }

 private:
  ros::Publisher jointPublisher_;
};

}  // namespace double_slit
}  // namespace ocs2
