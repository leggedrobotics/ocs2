#include "ocs2_ballbot_mpcnet/BallbotMpcnetDefinition.h"

namespace ocs2 {
namespace ballbot {

vector_t BallbotMpcnetDefinition::getGeneralizedTime(scalar_t t, const ModeSchedule& modeSchedule) {
  return t * vector_t::Ones(1);
}

vector_t BallbotMpcnetDefinition::getRelativeState(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories) {
  vector_t relativeState = x - targetTrajectories.getDesiredState(t);
  Eigen::Matrix<scalar_t, 2, 2> R = (Eigen::Matrix<scalar_t, 2, 2>() << cos(x(2)), -sin(x(2)), sin(x(2)), cos(x(2))).finished().transpose();
  relativeState.segment<2>(0) = R * relativeState.segment<2>(0);
  relativeState.segment<2>(5) = R * relativeState.segment<2>(5);
  return relativeState;
}

matrix_t BallbotMpcnetDefinition::getInputTransformation(scalar_t t, const vector_t& x) {
  return matrix_t::Identity(3, 3);
}

bool BallbotMpcnetDefinition::validState(const vector_t& x) {
  return true;
}

}  // namespace ballbot
}  // namespace ocs2
