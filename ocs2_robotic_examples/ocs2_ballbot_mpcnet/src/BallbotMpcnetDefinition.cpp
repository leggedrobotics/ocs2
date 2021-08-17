#include "ocs2_ballbot_mpcnet/BallbotMpcnetDefinition.h"

namespace ocs2 {
namespace ballbot {

vector_t BallbotMpcnetDefinition::getGeneralizedTime(scalar_t t, const ModeSchedule& modeSchedule) {
  return t * vector_t::Ones(1);
}

vector_t BallbotMpcnetDefinition::getRelativeState(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories) {
  return x - targetTrajectories.getDesiredState(t);
}

bool BallbotMpcnetDefinition::validState(const vector_t& x) {
  return true;
}

}  // namespace ballbot
}  // namespace ocs2
