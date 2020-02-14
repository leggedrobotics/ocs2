#pragma once

#include <Eigen/Core>
#include <raisim/World.hpp>

#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_core/Dimensions.h>

namespace ocs2 {
namespace ballbot {

/**
 * @brief Conversions between raiSim and ocs2 for the ballbot system
 *
 * Ballbot state: [ball x, ball y, base yaw, base pitch, base roll; <time derivatives of all positions>]
 * Ballbot input: [3-d ball torque]
 * raisim q: [3-d base position, base quaternion (w x y z), quaternion of spherical joint (w x y z)]
 * raisim dq: [3-d base linear vel in world, 3-d base angular vel in world, 3-d spherical joint velocities]
 * raisim generalizedForce [ 6-d wrench on base, 3-d torque through spherical joint]
 */
class BallbotRaisimConversions final {
 public:
  using dim_t = Dimensions<STATE_DIM_, INPUT_DIM_>;
  using state_vector_t = dim_t::state_vector_t;
  using input_vector_t = dim_t::input_vector_t;

  BallbotRaisimConversions();
  ~BallbotRaisimConversions() = default;

  Eigen::Vector3d ballCenterInWorld(const Eigen::VectorXd& q) const;

  std::pair<Eigen::VectorXd, Eigen::VectorXd> stateToRaisimGenCoordGenVel(const state_vector_t& state, const input_vector_t& input) const;

  state_vector_t raisimGenCoordGenVelToState(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const;

  Eigen::VectorXd inputToRaisimGeneralizedForce(double time, const input_vector_t& input, const state_vector_t& state,
                                                const Eigen::VectorXd& q, const Eigen::VectorXd& dq) const;

  static Eigen::Vector3d angularVelocityInWorldToEulerAngleDerivatives(const Eigen::Vector3d& eulerAngles,
                                                                       const Eigen::Vector3d& omega_world_base_inWorld);

  static Eigen::Vector3d eulerAngleDerivativesToAngularVelocityInWorld(const Eigen::Vector3d& eulerAngles,
                                                                       const Eigen::Vector3d& eulerAnglesTimeDerivative);

 private:
  double ballRadius_;
  double omniWheelRadius_;
  double distanceBaseToBallCenter_;
};

}  // namespace ballbot
}  // namespace ocs2
