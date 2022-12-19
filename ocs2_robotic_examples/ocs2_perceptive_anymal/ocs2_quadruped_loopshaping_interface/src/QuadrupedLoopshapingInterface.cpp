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
  const auto stanceFlags = switched_model::constantFeetArray(true);
  const auto uSystemForWeightCompensation =
      weightCompensatingInputs(getQuadrupedInterface().getComModel(), stanceFlags, switched_model::vector3_t::Zero());

  ocs2::vector_t initialFilterState;
  ocs2::vector_t initialFilterInput;
  this->getLoopshapingDefinition()->getFilterEquilibrium(uSystemForWeightCompensation, initialFilterState, initialFilterInput);
  initialState_ =
      this->getLoopshapingDefinition()->concatenateSystemAndFilterState(getQuadrupedInterface().getInitialState(), initialFilterState);

  // wrap with loopshaping
  timeTriggeredRolloutPtr_.reset(
      new ocs2::TimeTriggeredRollout(*getOptimalControlProblem().dynamicsPtr, getQuadrupedInterface().rolloutSettings()));
  loopshapingSynchronizedModule_ = std::make_shared<ocs2::LoopshapingSynchronizedModule>(this->getLoopshapingDefinition(),
                                                                                         getQuadrupedInterface().getSynchronizedModules());
}

}  // namespace switched_model_loopshaping
