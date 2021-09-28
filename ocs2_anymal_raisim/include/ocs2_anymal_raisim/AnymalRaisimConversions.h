#pragma once

#include <Eigen/Core>
#include <raisim/World.hpp>
#include <utility>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/WholebodyDynamics.h>

namespace anymal {

/**
 * @brief Conversion helpers between OCS2 and RAIsim conventions
 * @note ocs2RbdState is: rpy [0-2], xyz [3-5] , qJoints [6-17], baseLocalVelocities [18-23], dqJoints [24-35]
 * @note Raisim q: position (x y z), quaternion (w x y z), joint positions
 * @note Raisim dq: linear vel in world, angular vel in world, joint vel
 */
class AnymalRaisimConversions {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using com_model_t = switched_model::ComModelBase<double>;
  using kinematic_model_t = switched_model::KinematicsModelBase<double>;
  using wholebody_model_t = switched_model::WholebodyDynamics<double>;

  /**
   * @brief Constructor
   */
  AnymalRaisimConversions(const com_model_t& comModel, const kinematic_model_t& kinematicModel, const wholebody_model_t& wholebodyModel)
      : kinematicModelPtr_(kinematicModel.clone()), wholebodyModelPtr_(wholebodyModel.clone()) {}

  /**
   * @brief Convert ocs2 anymal state to generalized coordinate and generalized velocity used by RAIsim
   * @param[in] state: the state to be converted
   * @param[in] input: current input (includes state-information due to the kinematic leg model)
   * @return {q, dq} pair that represents the state
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> stateToRaisimGenCoordGenVel(const state_vector_t& state, const input_vector_t& input) const;

  /**
   * @brief Convert RAIsim generalized coordinates and velocities to ocs2 anymal RBD state
   * @param[in] q the generalized coordinate
   * @param[in] dq the generalized velocity
   * @return the corresponding anymal RBD state
   */
  switched_model::rbd_state_t raisimGenCoordGenVelToRbdState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const;

  /**
   * @brief Convert RAIsim generalized coordinates and velocities to ocs2 anymal state
   * @note This should be the inverse to stateToRaisimGenCoordGenVel
   * @param[in] q the generalized coordinate
   * @param[in] dq the generalized velocity
   * @return the corresponding ocs2 anymal state
   */
  state_vector_t raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const;

  /**
   * @brief Convert ocs2 control input to RAIsim generalized force
   * @param[in] time: The current time
   * @param[in] input: The control computed by the ocs2 controller
   * @param[in] state: The current state
   * @param[in] q: The current raisim generalized coordinate
   * @param[in] dq: The current raisim generalized velocity
   * @return The generalized forces to be applied to the system
   */
  Eigen::VectorXd inputToRaisimGeneralizedForce(double time, const input_vector_t& input, const state_vector_t& state,
                                                const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const;

  /**
   * @brief Convert ocs2 control input to RAIsim PD setpoints
   * @param[in] time: The current time
   * @param[in] input: The control computed by the ocs2 controller
   * @param[in] state: The current state
   * @param[in] q: The current raisim generalized coordinate
   * @param[in] dq: The current raisim generalized velocity
   * @return The generalized position and velocities to be used as PD control setpoints by RAIsim
   */
  static std::pair<Eigen::VectorXd, Eigen::VectorXd> inputToRaisimPdTargets(double time, const input_vector_t& input,
                                                                            const state_vector_t& state, const Eigen::VectorXd& q,
                                                                            const Eigen::VectorXd& dq);

  /**
   * @brief extractModelData
   * @param time[in] current time
   * @param sys[in] handle to the anymal system inside RAIsim
   */
  void extractModelData(double time, const raisim::ArticulatedSystem& sys);

 public:
  raisim::HeightMap const* terrain_ = nullptr;

 private:
  std::unique_ptr<const kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<const wholebody_model_t> wholebodyModelPtr_;
};
}  // namespace anymal
