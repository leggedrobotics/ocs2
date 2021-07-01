
#include "ocs2_switched_model_interface/cost/SwitchedModelCostBase.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

namespace switched_model {

SwitchedModelCostBase::SwitchedModelCostBase(const MotionTrackingCost::Weights& trackingWeights,
                                             const SwitchedModelModeScheduleManager& modeScheduleManager,
                                             const SwingTrajectoryPlanner& swingTrajectoryPlanner, const kinematic_model_t& kinematicModel,
                                             const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                             const ad_com_model_t& adComModel, ModelSettings options)
    : ocs2::CostFunctionBase(),
      comModelPtr_(comModel.clone()),
      trackingCostPtr_(new MotionTrackingCost(trackingWeights, modeScheduleManager, kinematicModel, adKinematicModel, comModel, adComModel,
                                              options.recompileLibraries_)),
      footPlacementCost_(new FootPlacementCost(FootPlacementCostParameters(options.mu_, options.delta_),
                                               FootPlacementCostParameters(options.muSdf_, options.deltaSdf_), adComModel, adKinematicModel,
                                               options.recompileLibraries_)),
      modeScheduleManagerPtr_(&modeScheduleManager),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner){};

SwitchedModelCostBase::SwitchedModelCostBase(const SwitchedModelCostBase& rhs)
    : ocs2::CostFunctionBase(rhs),
      comModelPtr_(rhs.comModelPtr_->clone()),
      footPlacementCost_(rhs.footPlacementCost_->clone()),
      trackingCostPtr_(rhs.trackingCostPtr_->clone()),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_) {}

SwitchedModelCostBase* SwitchedModelCostBase::clone() const {
  return new SwitchedModelCostBase(*this);
}

scalar_t SwitchedModelCostBase::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  update(t, x, u);

  return trackingCostPtr_->getValue(t, x, u, *costDesiredTrajectoriesPtr_) + footPlacementCost_->getCostValue();
}

ScalarFunctionQuadraticApproximation SwitchedModelCostBase::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[SwitchedModelCostBase] costDesiredTrajectoriesPtr_ is not set");
  }

  update(t, x, u);

  ScalarFunctionQuadraticApproximation L = trackingCostPtr_->getQuadraticApproximation(t, x, u, *costDesiredTrajectoriesPtr_);
  L.f += footPlacementCost_->getCostValue();
  L.dfdx += footPlacementCost_->getCostDerivativeState();
  L.dfdxx += footPlacementCost_->getCostSecondDerivativeState();
  return L;
}

void SwitchedModelCostBase::update(scalar_t t, const vector_t& x, const vector_t& u) {
  // Foot placement costs
  feet_array_t<const FootTangentialConstraintMatrix*> constraints = {{nullptr}};
  feet_array_t<SignedDistanceConstraint> sdfConstraints = {{nullptr, 0.0}};
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    const auto& footPhase = swingTrajectoryPlannerPtr_->getFootPhase(leg, t);
    constraints[leg] = footPhase.getFootTangentialConstraintInWorldFrame();
    sdfConstraints[leg] = footPhase.getSignedDistanceConstraint(t);
  }
  footPlacementCost_->setStateAndConstraint(x, constraints, sdfConstraints);
}

}  // namespace switched_model
