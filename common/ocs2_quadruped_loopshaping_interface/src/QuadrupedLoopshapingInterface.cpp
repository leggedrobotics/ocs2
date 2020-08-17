//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"
#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"

namespace switched_model_loopshaping {

QuadrupedLoopshapingInterface::QuadrupedLoopshapingInterface(std::unique_ptr<switched_model::QuadrupedInterface> quadrupedPtr,
                                                             const std::string& pathToConfigFolder)
    : quadrupedPtr_(std::move(quadrupedPtr)) {
  // Load loopshaping
  loopshapingDefinition_ = ocs2::loopshaping_property_tree::load(pathToConfigFolder + "/loopshaping.info");
  loopshapingDefinition_->print();
  filterDynamicsPtr_.reset(new ocs2::LoopshapingFilterDynamics(loopshapingDefinition_));

  // Initialize state including filter state
  const auto totalWeight = quadrupedPtr_->getComModel().totalMass() * 9.81;
  vector_t uSystemForWeightCompensation = vector_t::Zero(SYSTEM_INPUT_DIM);
  size_t numLegs(4);
  for (size_t i = 0; i < numLegs; i++) {
    uSystemForWeightCompensation(3 * i + 2) = totalWeight / numLegs;
  }

  ocs2::vector_t initialFilterState;
  ocs2::vector_t initialFilterInput;
  loopshapingDefinition_->getFilterEquilibrium(uSystemForWeightCompensation, initialFilterState, initialFilterInput);
  initialState_ = loopshapingDefinition_->concatenateSystemAndFilterState(quadrupedPtr_->getInitialState(), initialFilterState);

  // Wrap with loopshaping
  dynamicsPtr_ = ocs2::LoopshapingDynamics::create(quadrupedPtr_->getDynamics(), loopshapingDefinition_);
  constraintsPtr_ = ocs2::LoopshapingConstraint::create(*quadrupedPtr_->getConstraintPtr(), loopshapingDefinition_);
  costFunctionPtr_ = ocs2::LoopshapingCost::create(quadrupedPtr_->getCost(), loopshapingDefinition_);
  operatingPointsPtr_.reset(new ocs2::LoopshapingOperatingPoint(quadrupedPtr_->getOperatingPoints(), loopshapingDefinition_));

  timeTriggeredRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*dynamicsPtr_, quadrupedPtr_->rolloutSettings()));

  loopshapingModeScheduleManager_ =
      std::make_shared<LoopshapingModeScheduleManager>(quadrupedPtr_->getModeScheduleManagerPtr(), loopshapingDefinition_);
  loopshapingSynchronizedModule_ =
      std::make_shared<LoopshapingSynchronizedModule>(quadrupedPtr_->getSynchronizedModules(), loopshapingDefinition_);
}

}  // namespace switched_model_loopshaping
