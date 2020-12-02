//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingInterface.h"
#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"

namespace switched_model_loopshaping {

QuadrupedLoopshapingInterface::QuadrupedLoopshapingInterface(std::unique_ptr<switched_model::QuadrupedInterface> quadrupedPtr,
                                                             std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition)
    : ocs2::LoopshapingRobotInterface(std::move(quadrupedPtr), std::move(loopshapingDefinition)) {
  // initialize state including filter state
  const auto totalWeight = getQuadrupedInterface().getComModel().totalMass() * 9.81;
  vector_t uSystemForWeightCompensation = vector_t::Zero(SYSTEM_INPUT_DIM);
  const size_t numLegs = 4;
  for (size_t i = 0; i < numLegs; i++) {
    uSystemForWeightCompensation(3 * i + 2) = totalWeight / numLegs;
  }

  ocs2::vector_t initialFilterState;
  ocs2::vector_t initialFilterInput;
  this->getLoopshapingDefinition()->getFilterEquilibrium(uSystemForWeightCompensation, initialFilterState, initialFilterInput);
  initialState_ =
      this->getLoopshapingDefinition()->concatenateSystemAndFilterState(getQuadrupedInterface().getInitialState(), initialFilterState);

  // wrap with loopshaping
  timeTriggeredRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(getDynamics(), getQuadrupedInterface().rolloutSettings()));
  loopshapingSynchronizedModule_ = std::make_shared<ocs2::LoopshapingSynchronizedModule>(this->getLoopshapingDefinition(),
                                                                                         getQuadrupedInterface().getSynchronizedModules());
}

}  // namespace switched_model_loopshaping
