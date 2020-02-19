//
// Created by rgrandia on 13.02.19.
//

#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_core/Dimensions.h>
#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/DummyObserver.h>

namespace switched_model {

class QuadrupedXppVisualizer : public ocs2::DummyObserver<STATE_DIM, INPUT_DIM> {
 public:
  using BASE = ocs2::DummyObserver<STATE_DIM, INPUT_DIM>;
  using typename BASE::command_data_t;
  using typename BASE::primal_solution_t;
  using typename BASE::system_observation_t;

  using dimension_t = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimension_t::scalar_t;
  using state_vector_t = typename dimension_t::state_vector_t;
  using scalar_array_t = typename dimension_t::scalar_array_t;
  using size_array_t = typename dimension_t::size_array_t;
  using state_vector_array_t = typename dimension_t::state_vector_array_t;
  using input_vector_t = typename dimension_t::input_vector_t;

  using rbd_state_vector_t = Eigen::Matrix<scalar_t, RBD_STATE_DIM, 1>;

  using cost_desired_trajectories_t = ocs2::CostDesiredTrajectories;

  using system_observation_array_t = std::vector<system_observation_t, Eigen::aligned_allocator<system_observation_t>>;
  using vector_3d_t = Eigen::Matrix<scalar_t, 3, 1>;
  using vector_3d_array_t = std::array<vector_3d_t, 4>;

  using com_model_t = ComModelBase<double>;
  using kinematic_model_t = KinematicsModelBase<double>;

  /**
   *
   * @param kinematicModel
   * @param comModel
   * @param n
   * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
   */
  QuadrupedXppVisualizer(const kinematic_model_t& kinematicModel, const com_model_t& comModel, ros::NodeHandle& n,
                         double maxUpdateFrequency = 50.0)
      : kinematicModelPtr_(kinematicModel.clone()), comModelPtr_(comModel.clone()), minPublishTimeDifference_(1 / maxUpdateFrequency) {
    launchVisualizerNode(n);
  };

  ~QuadrupedXppVisualizer() override = default;

  void update(const system_observation_t& observation, const primal_solution_t& primalSolution, const command_data_t& command) override {
    static scalar_t lastTime = std::numeric_limits<scalar_t>::lowest();
    if (observation.time() - lastTime > minPublishTimeDifference_) {
      publishObservation(observation);
      publishDesiredTrajectory(command.mpcCostDesiredTrajectories_);
      publishOptimizedStateTrajectory(primalSolution.timeTrajectory_, primalSolution.stateTrajectory_, primalSolution.eventTimes_,
                                      primalSolution.subsystemsSequence_);
      lastTime = observation.time();
    }
  }

  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishObservation(const system_observation_t& observation);

  void publishTrajectory(const system_observation_array_t& system_observation_array, double speed = 1.0);

  void publishDesiredTrajectory(const cost_desired_trajectories_t& costDesiredTrajectory);

  void publishOptimizedStateTrajectory(const scalar_array_t& mpcTimeTrajectory, const state_vector_array_t& mpcStateTrajectory,
                                       const scalar_array_t& eventTimes, const size_array_t& subsystemSequence);

 private:
  /**
   * Publishes the xpp visualization messages.
   *
   * @param time: time.
   * @param basePose: Base pose in the origin frame.
   * @param baseLocalVelocities: Base local velocities.
   * @param feetPosition: Feet position in the origin frame.
   * @param feetVelocity: Feet velocity in the origin frame.
   * @param feetAcceleration: Feet acceleration in the origin frame.
   * @param feetForce: Contact forces acting on the feet in the origin frame.
   */
  void publishXppVisualizer(scalar_t time, const contact_flag_t& contactFlags, const base_coordinate_t& basePose,
                            const base_coordinate_t& baseLocalVelocities, const joint_coordinate_t& jointAngles,
                            const vector_3d_array_t& feetPosition, const vector_3d_array_t& feetVelocity,
                            const vector_3d_array_t& feetAcceleration, const vector_3d_array_t& feetForce);

  void computeFeetState(const state_vector_t& state, const input_vector_t& input, vector_3d_array_t& o_feetPosition,
                        vector_3d_array_t& o_feetVelocity, vector_3d_array_t& o_contactForces);

  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;

  tf::TransformBroadcaster tf_broadcaster_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

  ros::Publisher costDesiredPublisher_;
  ros::Publisher stateOptimizedPublisher_;
  ros::Publisher feetOptimizedPublisher_;
  ros::Publisher rvizMarkerPub_;
  ros::Time startTime_;

  double minPublishTimeDifference_;
};

}  // namespace switched_model
