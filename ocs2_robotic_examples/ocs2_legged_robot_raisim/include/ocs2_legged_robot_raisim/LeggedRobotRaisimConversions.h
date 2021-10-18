#pragma once

#include <Eigen/Core>
#include <raisim/object/terrain/HeightMap.hpp>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/PinocchioCentroidalInverseDynamicsPD.h>

#include <ocs2_legged_robot/common/ModelSettings.h>

namespace ocs2 {
namespace legged_robot {

/**
 * Conversions between RaiSim variables and ocs2 variables for the legged robot.
 */
class LeggedRobotRaisimConversions {
 public:
  /**
   * Constructor.
   * @param [in] pinocchioInterface : The predefined pinocchio interface for the robot.
   * @param [in] centroidalModelInfo : The centroidal model information.
   * @param [in] modelSettings : The model settings.
   */
  LeggedRobotRaisimConversions(PinocchioInterface& pinocchioInterface, CentroidalModelInfo centroidalModelInfo, ModelSettings modelSettings)
      : centroidalModelRbdConversionsPtr_(new CentroidalModelRbdConversions(pinocchioInterface, centroidalModelInfo)),
        pinocchioCentroidalInverseDynamicsPDPtr_(
            new PinocchioCentroidalInverseDynamicsPD(pinocchioInterface, centroidalModelInfo, modelSettings.contactNames3DoF)) {}

  /**
   * Default destructor.
   */
  ~LeggedRobotRaisimConversions() = default;

  /**
   * @brief Convert ocs2 state to generalized coordinate and generalized velocity used by RaiSim.
   * @param [in] state : The state to be converted.
   * @param [in] input : The current input (includes state-information due to the kinematic leg model).
   * @return The {q, dq} pair that represents the simulator state.
   */
  std::pair<Eigen::VectorXd, Eigen::VectorXd> stateToRaisimGenCoordGenVel(const vector_t& state, const vector_t& input);

  /**
   * @brief Convert RaiSim generalized coordinates and velocities to ocs2 state.
   * @param [in] q : The generalized coordinate.
   * @param [in] dq : The generalized velocity.
   * @return The corresponding ocs2 state.
   */
  vector_t raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

  /**
   * @brief Convert ocs2 control input to RaiSim generalized force.
   * @param [in] time : The current time.
   * @param [in] input : The control computed by the ocs2 controller.
   * @param [in] state : The current state.
   * @param [in] q : The current raisim generalized coordinate.
   * @param [in] dq : The current raisim generalized velocity.
   * @return The generalized forces to be applied to the system.
   */
  Eigen::VectorXd inputToRaisimGeneralizedForce(double time, const vector_t& input, const vector_t& state, const Eigen::VectorXd& q,
                                                const Eigen::VectorXd& dq);

  /**
   * @brief Set the PD gains.
   * @param [in] pGains : The proportional gains.
   * @param [in] dGains : The derivative gains.
   */
  void setGains(const vector_t& pGains, const vector_t& dGains) { pinocchioCentroidalInverseDynamicsPDPtr_->setGains(pGains, dGains); }

 protected:
  /**
   * @brief Convert RaiSim generalized coordinates and velocities to ocs2 RBD state.
   * @param [in] q : The generalized coordinate.
   * @param [in] dq : The generalized velocity.
   * @return The corresponding RBD state.
   */
  vector_t raisimGenCoordGenVelToRbdState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

  /**
   * @brief Convert RaiSim generalized coordinates and velocities to ocs2 input.
   * @param [in] q : The generalized coordinate.
   * @param [in] dq : The generalized velocity.
   * @return The corresponding ocs2 input (includes state-information due to the kinematic leg model).
   */
  vector_t raisimGenCoordGenVelToInput(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);

  /**
   * @brief Convert RaiSim joint variables to ocs2 joint variables.
   * @note This can be removed if changing the RaiSim joint order works eventually.
   * @param [in] raisimJoint : The joint variables in RaiSim order.
   * @return The joint variables in ocs2 order.
   */
  vector_t raisimJointOrderToOcs2JointOrder(const Eigen::VectorXd& raisimJoint);

  /**
   * @brief Convert ocs2 joint variables to RaiSim joint variables.
   * @note This can be removed if changing the RaiSim joint order works eventually.
   * @param [in] ocs2Joint : The joint variables in ocs2 order.
   * @return The joint variables in RaiSim order.
   */
  Eigen::VectorXd ocs2JointOrderToRaisimJointOrder(const vector_t& ocs2Joint);

  /**
   * @brief Find yaw angle that is closest to continuous reference yaw angle.
   * @note Copied from ocs2_anymal_commands/TerrainAdaptation.h.
   * @param [in] yaw : The yaw angle from the unique Euler angles.
   * @param [in] reference : The continuous reference yaw angle.
   * @return A yaw angle (yaw + k*2*pi) with k such that the result is within [reference - pi, reference + pi].
   */
  scalar_t findOrientationClostestToReference(scalar_t yaw, scalar_t reference);

 public:
  raisim::HeightMap const* terrain_ = nullptr;

 private:
  Eigen::Vector3d continuousOrientation_;
  std::unique_ptr<CentroidalModelRbdConversions> centroidalModelRbdConversionsPtr_;
  std::unique_ptr<PinocchioCentroidalInverseDynamicsPD> pinocchioCentroidalInverseDynamicsPDPtr_;
};

}  // namespace legged_robot
}  // namespace ocs2
