#include "ocs2_switched_model_interface/initialization/ComKinoInitializer.h"

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

ComKinoInitializer::ComKinoInitializer(const com_model_t& comModel, const SwitchedModelModeScheduleManager& modeScheduleManager,
                                       const ocs2::StateInputConstraintCollection& equalityConstraints,
                                       const ocs2::PreComputation& preComputation)
    : comModelPtr_(comModel.clone()),
      modeScheduleManagerPtr_(&modeScheduleManager),
      equalityConstraintPtr_(equalityConstraints.clone()),
      preComputationPtr_(preComputation.clone()) {}

ComKinoInitializer::ComKinoInitializer(const ComKinoInitializer& rhs)
    : ocs2::Initializer(rhs),
      comModelPtr_(rhs.comModelPtr_->clone()),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_),
      equalityConstraintPtr_(rhs.equalityConstraintPtr_->clone()),
      preComputationPtr_(rhs.preComputationPtr_->clone()) {}

ComKinoInitializer* ComKinoInitializer::clone() const {
  return new ComKinoInitializer(*this);
}

void ComKinoInitializer::compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const comkino_state_t comkinoState = state;
  const auto basePose = getBasePose(comkinoState);
  const auto contactFlags = modeScheduleManagerPtr_->getContactFlags(time);

  // Initial guess
  input = weightCompensatingInputs(*comModelPtr_, contactFlags, getOrientation(basePose));

  // Project inputs onto state input equality constraints
  if (!equalityConstraintPtr_->empty()) {
    // Precomputation
    constexpr auto request = ocs2::Request::Constraint + ocs2::Request::Approximation;
    preComputationPtr_->request(request, time, state, input);

    // C_{k} * dx_{k} + D_{k} * du_{k} + e_{k} = 0
    const auto constraints = equalityConstraintPtr_->getLinearApproximation(time, state, input, *preComputationPtr_);
    if (constraints.f.size() > 0) {
      // solve D_{k} * du_{k} = -e_{k} in a least square sense w.r.t du. (fix the constraints with dx = 0)
      // du_{k} = -D_{k}' inv(D_{k}*D_{k}') e_{k}
      Eigen::LDLT<ocs2::matrix_t> ldlt(constraints.dfdu * constraints.dfdu.transpose());
      vector_t tmp = ldlt.solve(constraints.f);
      input.noalias() -= constraints.dfdu.transpose() * tmp;
    }
  }

  nextState = state;
}

}  // namespace switched_model
