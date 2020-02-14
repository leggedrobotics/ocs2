//
// Created by rgrandia on 13.02.19.
//

#pragma once

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/RobotStateJoint.h>
#include <xpp_msgs/topic_names.h>

#include <ocs2_comm_interfaces/SystemObservation.h>
#include <ocs2_core/Dimensions.h>
#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
class QuadrupedXppVisualizer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr auto rbd_state_dim_ = 12 + 2 * JOINT_COORD_SIZE;

  using dimension_t = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimension_t::scalar_t;
  using state_vector_t = typename dimension_t::state_vector_t;
  using scalar_array_t = typename dimension_t::scalar_array_t;
  using state_vector_array_t = typename dimension_t::state_vector_array_t;
  using input_vector_t = typename dimension_t::input_vector_t;

  using rbd_state_vector_t = Eigen::Matrix<scalar_t, rbd_state_dim_, 1>;

  using cost_desired_trajectories_t = ocs2::CostDesiredTrajectories;

  using system_observation_t = ocs2::SystemObservation<STATE_DIM, INPUT_DIM>;
  using system_observation_array_t = std::vector<system_observation_t, Eigen::aligned_allocator<system_observation_t>>;
  using vector_3d_t = Eigen::Matrix<scalar_t, 3, 1>;
  using vector_3d_array_t = std::array<vector_3d_t, 4>;

  using com_model_t = ComModelBase<double>;
  using kinematic_model_t = KinematicsModelBase<double>;

  QuadrupedXppVisualizer(const kinematic_model_t& kinematicModel, const com_model_t& comModel, std::string robotName,
                         bool save_rosbag = false)
      : kinematicModelPtr_(kinematicModel.clone()), comModelPtr_(comModel.clone()), robotName_(std::move(robotName)), save_rosbag_(save_rosbag) {
    if (save_rosbag_) {
      rosbagFile_ = "ocs2_" + robotName_ + "_traj.bag";
      bag_.open(rosbagFile_, rosbag::bagmode::Write);
    }
  };

  ~QuadrupedXppVisualizer() {
    if (save_rosbag_) {
      ROS_INFO_STREAM("ROS Bag file is being saved to \"" + rosbagFile_ + "\" ...");
      try {
        bag_.write("xpp/trajectory_des", startTime_, robotStateCartesianTrajectoryMsg_);
      } catch (const rosbag::BagException& err) {
        std::cerr << "Error writing rosbag message: " << err.what() << std::endl;
      }
      bag_.close();
      ROS_INFO_STREAM("ROS Bag file is saved.");
    }
  }

  /**
   * Launches the visualization node
   *
   * @param [in] argc: command line number of inputs.
   * @param [in] argv: command line inputs' value.
   */
  void launchVisualizerNode(int argc, char* argv[]);

  /**
   * Visualizes the current observation.
   *
   * @param [in] observation: The current observation.
   */
  void publishObservation(const system_observation_t& observation);

  /**
   * Visualize trajectory of observations.
   *
   * @param [in] observation_array: Array of observations.
   */
  void publishTrajectory(const system_observation_array_t& system_observation_array, double speed = 1.0);

  void publishDesiredTrajectory(scalar_t startTime, const cost_desired_trajectories_t& costDesiredTrajectory);

  void publishOptimizedStateTrajectory(const scalar_array_t& mpcTimeTrajectory, const state_vector_array_t& mpcStateTrajectory);

 private:
  /**
   * Publishes the xpp visualization messages and also if "SAVE_ROS_BAG" is defined, it saves then in a ROS bag file.
   *
   * @param time: time.
   * @param basePose: Base pose in the origin frame.
   * @param baseLocalVelocities: Base local velocities.
   * @param feetPosition: Feet position in the origin frame.
   * @param feetVelocity: Feet velocity in the origin frame.
   * @param feetAcceleration: Feet acceleration in the origin frame.
   * @param feetForce: Contact forces acting on the feet in the origin frame.
   */
  void publishXppVisualizer(const scalar_t& time, const base_coordinate_t& basePose, const base_coordinate_t& baseLocalVelocities,
                            const joint_coordinate_t& jointAngles, const vector_3d_array_t& feetPosition,
                            const vector_3d_array_t& feetVelocity, const vector_3d_array_t& feetAcceleration,
                            const vector_3d_array_t& feetForce);

  void computeFeetState(const state_vector_t& state, const input_vector_t& input, vector_3d_array_t& o_feetPosition,
                        vector_3d_array_t& o_feetVelocity, vector_3d_array_t& o_contactForces);

  static void sigintHandler(int sig);

  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;

  bool save_rosbag_;
  std::string robotName_;
  std::string rosbagFile_;

  ros::Publisher visualizationPublisher_;
  ros::Publisher visualizationJointPublisher_;
  ros::Publisher costDesiredPublisher_;
  ros::Publisher stateOptimizedPublisher_;
  ros::Publisher feetOptimizedPublisher_;
  ros::Time startTime_;

  rosbag::Bag bag_;
  xpp_msgs::RobotStateCartesianTrajectory robotStateCartesianTrajectoryMsg_;
};

}  // namespace switched_model

#include "implementation/QuadrupedXppVisualizer.h"

