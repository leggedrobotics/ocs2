#include "ocs2_legged_robot_mpcnet/LeggedRobotMpcnetDefinition.h"

#include <iostream>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "ocs2_legged_robot_mpcnet/helper/Logic.h"

namespace ocs2 {
namespace legged_robot {

vector_t LeggedRobotMpcnetDefinition::getGeneralizedTime(scalar_t t, const ModeSchedule& modeSchedule) {
  feet_array_t<LegPhase> swingPhasePerLeg = getSwingPhasePerLeg(t, modeSchedule);
  vector_t generalizedTime;
  generalizedTime.resize(3 * NUM_CONTACT_POINTS, Eigen::NoChange);
  // phase
  for (int i = 0 * NUM_CONTACT_POINTS; i < 1 * NUM_CONTACT_POINTS; i++) {
    if (swingPhasePerLeg[i % NUM_CONTACT_POINTS].phase < 0.0) {
      generalizedTime[i] = 0.0;
    } else {
      generalizedTime[i] = swingPhasePerLeg[i % NUM_CONTACT_POINTS].phase;
    }
  }
  // phase rate
  for (int i = 1 * NUM_CONTACT_POINTS; i < 2 * NUM_CONTACT_POINTS; i++) {
    if (swingPhasePerLeg[i % NUM_CONTACT_POINTS].phase < 0.0) {
      generalizedTime[i] = 0.0;
    } else {
      generalizedTime[i] = 1.0 / swingPhasePerLeg[i % NUM_CONTACT_POINTS].duration;
    }
  }
  // sin(pi * phase)
  for (int i = 2 * NUM_CONTACT_POINTS; i < 3 * NUM_CONTACT_POINTS; i++) {
    if (swingPhasePerLeg[i % NUM_CONTACT_POINTS].phase < 0.0) {
      generalizedTime[i] = 0.0;
    } else {
      generalizedTime[i] = std::sin(M_PI * swingPhasePerLeg[i % NUM_CONTACT_POINTS].phase);
    }
  }
  return generalizedTime;
}

vector_t LeggedRobotMpcnetDefinition::getRelativeState(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories) {
  vector_t relativeState = x - targetTrajectories.getDesiredState(t);
  matrix3_t R = getRotationMatrixFromZyxEulerAngles<scalar_t>(x.segment<3>(9)).transpose();
  relativeState.segment<3>(6) = R * relativeState.segment<3>(6);
  relativeState.segment<3>(9) = R * relativeState.segment<3>(9);
  return relativeState;
}

bool LeggedRobotMpcnetDefinition::validState(const vector_t& x) {
  vector_t deviation = x - defaultState_;
  if (std::abs(deviation[8]) > 0.2) {
    std::cerr << "LeggedRobotMpcnetDefinition::validState Height diverged: " << x[8] << std::endl;
    return false;
  } else if (std::abs(deviation[10]) > 30.0 * M_PI / 180.0) {
    std::cerr << "LeggedRobotMpcnetDefinition::validState Pitch diverged: " << x[10] << std::endl;
    return false;
  } else if (std::abs(deviation[11]) > 30.0 * M_PI / 180.0) {
    std::cerr << "LeggedRobotMpcnetDefinition::validState Roll diverged: " << x[11] << std::endl;
    return false;
  } else {
    return true;
  }
}

}  // namespace legged_robot
}  // namespace ocs2
