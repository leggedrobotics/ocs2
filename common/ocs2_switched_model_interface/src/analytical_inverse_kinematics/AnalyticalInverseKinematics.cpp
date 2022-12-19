#include "ocs2_switched_model_interface/analytical_inverse_kinematics/AnalyticalInverseKinematics.h"

namespace switched_model {
namespace analytical_inverse_kinematics {

void getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                            const Eigen::Vector3d& positionBaseToFootInBaseFrame,
                                                            const LegInverseKinematicParameters& parameters, size_t limb) {
  Eigen::Vector3d positionHAAToFootInBaseFrame = positionBaseToFootInBaseFrame - parameters.positionBaseToHaaCenterInBaseFrame_;

  /// Rescaling target
  const double reachSquared{positionHAAToFootInBaseFrame.squaredNorm()};
  if (reachSquared > parameters.maxReachSquared_) {
    positionHAAToFootInBaseFrame.array() *= parameters.maxReach_ / std::sqrt(reachSquared);
  } else if (reachSquared < parameters.minReachSquared_ && reachSquared > 0.0) {
    positionHAAToFootInBaseFrame.array() *= parameters.minReach_ / std::sqrt(reachSquared);
  }

  /// Rescaling Yz
  double positionYzSquared{positionHAAToFootInBaseFrame.tail<2>().squaredNorm()};
  if (positionYzSquared < parameters.positionHipToFootYoffsetSquared_ && positionYzSquared > 0.0) {
    positionHAAToFootInBaseFrame.tail<2>().array() *= std::sqrt(parameters.positionHipToFootYoffsetSquared_ / positionYzSquared);
    positionYzSquared = parameters.positionHipToFootYoffsetSquared_;

    if (std::abs(positionHAAToFootInBaseFrame[0]) > parameters.maxReach_SP_ && positionHAAToFootInBaseFrame[0] != 0.0) {
      positionHAAToFootInBaseFrame[0] *= parameters.maxReach_SP_ / std::abs(positionHAAToFootInBaseFrame[0]);
    }
  }

  // HAA
  const double rSquared{std::max(0.0, positionYzSquared - parameters.positionHipToFootYoffsetSquared_)};
  const double r{std::sqrt(rSquared)};
  const double delta{std::atan2(positionHAAToFootInBaseFrame.y(), -positionHAAToFootInBaseFrame.z())};
  const double beta{std::atan2(r, parameters.positionHipToFootYoffset_)};
  const double qHAA{beta + delta - M_PI_2};
  legJoints[0] = qHAA;

  /// simplification for anymal
  const double l_squared{rSquared + positionHAAToFootInBaseFrame[0] * positionHAAToFootInBaseFrame[0]};
  const double l{std::sqrt(l_squared)};

  // Phi 1
  double cosphi1{0.5 * (parameters.a1_squared_ + l_squared - parameters.a2_squared_) / (parameters.a1_ * l)};
  cosphi1 = std::max(-1.0, std::min(cosphi1, 1.0));  // Clip to bounds of acos
  const double phi1{std::acos(cosphi1)};

  // Phi 2
  double cosphi2 = {0.5 * (parameters.a2_squared_ + l_squared - parameters.a1_squared_) / (parameters.a2_ * l)};
  cosphi2 = std::max(-1.0, std::min(cosphi2, 1.0));  // Clip to bounds of acos
  const double phi2{std::acos(cosphi2)};

  // HFE
  const double theta_prime{std::atan2(positionHAAToFootInBaseFrame[0], r)};
  double qHFE{phi1 - theta_prime};
  if (limb > 1) {
    qHFE = -phi1 - theta_prime;
  }
  legJoints[1] = qHFE;

  // KFE
  double qKFE = {phi1 + phi2 - parameters.KFEOffset_};
  if (limb < 2) {
    qKFE = -qKFE;
  }
  legJoints[2] = qKFE;
}

}  // namespace analytical_inverse_kinematics
}  // namespace switched_model
