#pragma once

#include <Eigen/Core>
#include <raisim/World.hpp>
#include <utility>

#include <ocs2_anymal_interface/OCS2AnymalInterface.h>
#include <ocs2_quadruped_interface/MRT_ROS_Quadruped.h>

namespace anymal {

//! Conversion helpers between OCS2 and RAIsim conventions
class AnymalRaisimConversions {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using mrt_t = switched_model::MRT_ROS_Quadruped<12>;

  /**
   * @brief Constructor
   * @param mrt: Shared pointer to an MRT instance, which is assumed to be updated by MPC
   */
  AnymalRaisimConversions(std::shared_ptr<mrt_t> mrt, std::shared_ptr<anymal::OCS2AnymalInterface> anymalInterface)
      : mrt_(std::move(mrt)), anymalInterface_(std::move(anymalInterface)) {}

  /**
   * @brief Convert ocs2 anymal state to generalized coordinate and generalized velocity used by RAIsim
   * @param[in] state: the state to be converted
   * @param[in] input: current input (includes state-information due to the kinematic leg model)
   * @return {q, dq} pair that represents the state
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> stateToRaisimGenCoordGenVel(const Eigen::Matrix<double, 24, 1>& state,
                                                                          const Eigen::Matrix<double, 24, 1>& input);

  /**
   * @brief Convert RAIsim generalized coordinates and velocities to ocs2 anymal state
   * @note This should be the inverse to stateToRaisimGenCoordGenVel
   * @param[in] q the generalized coordinate
   * @param[in] dq the generalized velocity
   * @return the corresponding ocs2 cart pole state
   */
  Eigen::Matrix<double, 24, 1> raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

  /**
   * @brief Convert ocs2 control input to RAIsim generalized force
   * @param[in] time: The current time
   * @param[in] input: The control computed by the ocs2 controller
   * @param[in] state: The current state
   * @return The generalized forces to be applied to the system
   */
  Eigen::VectorXd inputToRaisimGeneralizedForce(double time, const Eigen::Matrix<double, 24, 1>& input, const Eigen::Matrix<double, 24, 1>& state);

  /**
   * @brief extractModelData
   * @param time[in] current time
   * @param sys[in] handle to the anymal system inside RAIsim
   */
  void extractModelData(double time, const raisim::ArticulatedSystem& sys);

 protected:
  std::shared_ptr<mrt_t> mrt_;
  std::shared_ptr<anymal::OCS2AnymalInterface> anymalInterface_;
};
}  // namespace anymal
