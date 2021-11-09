#pragma once

#include <ros/ros.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

namespace ocs2 {

/**
 *  Dummy observer that publishes the current system observation that is required for some target command nodes.
 */
class MpcnetDummyObserverRos : public DummyObserver {
 public:
  /**
   * Constructor.
   * @param [in] nodeHandle : The ROS node handle.
   * @param [in] topicPrefix : The prefix defines the names for the observation's publishing topic "topicPrefix_mpc_observation".
   */
  explicit MpcnetDummyObserverRos(ros::NodeHandle& nodeHandle, std::string topicPrefix = "anonymousRobot");

  /**
   * Default destructor.
   */
  ~MpcnetDummyObserverRos() override = default;

  /**
   * Update and publish.
   * @param [in] observation: The current system observation.
   * @param [in] primalSolution: The current primal solution.
   * @param [in] command: The given command data.
   */
  void update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) override;

 private:
  ros::Publisher observationPublisher_;
};

}  // namespace ocs2
