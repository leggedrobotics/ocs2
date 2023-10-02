#include "ocs2_switched_model_interface/analytical_inverse_kinematics/LegInverseKinematicParameters.h"

namespace switched_model {
namespace analytical_inverse_kinematics {

LegInverseKinematicParameters::LegInverseKinematicParameters(const Eigen::Vector3d& positionBaseToHipInBaseFrame,
                                                             const Eigen::Vector3d& positionHipToThighInBaseFrame,
                                                             const Eigen::Vector3d& positionThighToShankInBaseFrame,
                                                             const Eigen::Vector3d& positionShankToFootInBaseFrame)
    : positionBaseToHipInBaseFrame_(positionBaseToHipInBaseFrame),
      positionHipToThighInBaseFrame_(positionHipToThighInBaseFrame),
      positionThighToShankInBaseFrame_(positionThighToShankInBaseFrame),
      positionShankToFootInBaseFrame_(positionShankToFootInBaseFrame) {
  initialize();
}

void LegInverseKinematicParameters::initialize() {
  // Store HAA center
  positionBaseToHaaCenterInBaseFrame_ = positionBaseToHipInBaseFrame_;
  positionBaseToHaaCenterInBaseFrame_[0] += positionHipToThighInBaseFrame_[0];

  // Store Hip Y offset
  positionHipToFootYoffset_ =
      positionHipToThighInBaseFrame_.y() + positionThighToShankInBaseFrame_.y() + positionShankToFootInBaseFrame_.y();
  positionHipToFootYoffsetSquared_ = positionHipToFootYoffset_ * positionHipToFootYoffset_;

  // Compute max / min extension limits
  const double thighToShankXZLength{
      Eigen::Vector3d(positionThighToShankInBaseFrame_.x(), 0.0, positionThighToShankInBaseFrame_.z()).norm()};
  const double shankToFootXZLength{Eigen::Vector3d(positionShankToFootInBaseFrame_.x(), 0.0, positionShankToFootInBaseFrame_.z()).norm()};
  minReach_SP_ = std::abs(shankToFootXZLength - thighToShankXZLength);
  maxReach_SP_ = std::abs(thighToShankXZLength + shankToFootXZLength);
  minReachSquared_ = positionHipToFootYoffsetSquared_ + minReach_SP_ * minReach_SP_;
  maxReachSquared_ = positionHipToFootYoffsetSquared_ + maxReach_SP_ * maxReach_SP_;
  minReach_ = std::sqrt(minReachSquared_);
  maxReach_ = std::sqrt(maxReachSquared_);

  // Trigonometry factors
  KFEOffset_ = std::abs(std::atan(positionShankToFootInBaseFrame_[0] / positionShankToFootInBaseFrame_[2]));
  a1_squared_ = positionThighToShankInBaseFrame_[0] * positionThighToShankInBaseFrame_[0] +
                positionThighToShankInBaseFrame_[2] * positionThighToShankInBaseFrame_[2];
  a1_ = std::sqrt(a1_squared_);
  a2_squared_ = positionShankToFootInBaseFrame_[0] * positionShankToFootInBaseFrame_[0] +
                positionShankToFootInBaseFrame_[2] * positionShankToFootInBaseFrame_[2];
  a2_ = std::sqrt(a2_squared_);
}

}  // namespace analytical_inverse_kinematics
}  // namespace switched_model