//
// Created by rgrandia on 09.06.21.
//

#include "ocs2_sqp/MultipleShootingInitialization.h"

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace multiple_shooting {

std::pair<vector_t, vector_t> initializeIntermediateNode(Initializer& initializer, scalar_t t, scalar_t t_next, const vector_t& x) {
  vector_t input, nextState;
  initializer.compute(t, x, t_next, input, nextState);
  return {input, nextState};
}

std::pair<vector_t, vector_t> initializeIntermediateNode(PrimalSolution& primalSolution, scalar_t t, scalar_t t_next, const vector_t& x,
                                                         bool useController) {
  // Use interpolation for next state
  const auto nextState = LinearInterpolation::interpolate(t_next, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_);

  // Determine input
  if (useController) {
    return {primalSolution.controllerPtr_->computeInput(t, x), nextState};
  } else {
    return {LinearInterpolation::interpolate(t, primalSolution.timeTrajectory_, primalSolution.inputTrajectory_), nextState};
  }
}

vector_t initializeEventNode(scalar_t t, const vector_t& x) {
  // Assume identity map for now
  return x;
}

}  // namespace multiple_shooting
}  // namespace ocs2