//
// Created by rgrandia on 22.09.21.
//

#include "ocs2_switched_model_interface/cost/FrictionConeCost.h"

namespace switched_model {

FrictionConeCost::FrictionConeCost(FrictionConeConstraint::Config config, int legNumber,
                                   const SwitchedModelModeScheduleManager& modeScheduleManager,
                                   std::unique_ptr<ocs2::PenaltyBase> penaltyFunction)
    : legNumber_(legNumber),
      constraint_(new FrictionConeConstraint(config, legNumber, modeScheduleManager)),
      penalty_(std::move(penaltyFunction)) {}

FrictionConeCost::FrictionConeCost(const FrictionConeCost& rhs)
    : legNumber_(rhs.legNumber_), constraint_(rhs.constraint_->clone()), penalty_(rhs.penalty_->clone()) {}

FrictionConeCost* FrictionConeCost::clone() const {
  return new FrictionConeCost(*this);
}

scalar_t FrictionConeCost::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                    const ocs2::TargetTrajectories& targetTrajectories, const ocs2::PreComputation& preComp) const {
  const auto h = constraint_->getValue(time, state, input, preComp);
  return penalty_->getValue(time, h(0));
}

ScalarFunctionQuadraticApproximation FrictionConeCost::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                 const vector_t& input,
                                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                                 const ocs2::PreComputation& preComp) const {
  auto h = constraint_->getQuadraticApproximation(time, state, input, preComp);

  const scalar_t penaltyValue = penalty_->getValue(time, h.f(0));
  const scalar_t penaltyDerivative = penalty_->getDerivative(time, h.f(0));
  const scalar_t penaltySecondDerivative = penalty_->getSecondDerivative(time, h.f(0));

  const vector3_t penaltySecondDev_dhdx = penaltySecondDerivative * h.dfdx.block<1, 3>(0, 0).transpose();

  // to make sure that dfdux in the state-only case has a right size
  ScalarFunctionQuadraticApproximation penaltyApproximation;

  int legIdx = 3 * legNumber_;

  penaltyApproximation.f = penaltyValue;

  penaltyApproximation.dfdx.setZero(STATE_DIM);
  penaltyApproximation.dfdx.segment<3>(0) = penaltyDerivative * h.dfdx.block<1, 3>(0, 0).transpose();

  penaltyApproximation.dfdu.setZero(INPUT_DIM);
  penaltyApproximation.dfdu.segment<3>(legIdx) = penaltyDerivative * h.dfdu.block<1, 3>(0, legIdx).transpose();

  penaltyApproximation.dfdxx = std::move(h.dfdxx[0]);
  penaltyApproximation.dfdxx.block<3, 3>(0, 0) *= penaltyDerivative;
  penaltyApproximation.dfdxx.block<3, 3>(0, 0).noalias() += penaltySecondDev_dhdx * h.dfdx.block<1, 3>(0, 0);

  penaltyApproximation.dfdux = std::move(h.dfdux[0]);
  penaltyApproximation.dfdux.block<3, 3>(3 * legNumber_, 0) *= penaltyDerivative;
  penaltyApproximation.dfdux.block<3, 3>(3 * legNumber_, 0).noalias() +=
      h.dfdu.block<1, 3>(0, legIdx).transpose() * penaltySecondDev_dhdx.transpose();

  penaltyApproximation.dfduu = std::move(h.dfduu[0]);
  penaltyApproximation.dfduu.block<3, 3>(legIdx, legIdx) *= penaltyDerivative;
  penaltyApproximation.dfduu.block<3, 3>(legIdx, legIdx).noalias() +=
      penaltySecondDerivative * h.dfdu.block<1, 3>(0, legIdx).transpose() * h.dfdu.block<1, 3>(0, legIdx);

  return penaltyApproximation;
}

}  // namespace switched_model
