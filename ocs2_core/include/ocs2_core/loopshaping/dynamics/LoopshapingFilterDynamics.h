
#pragma once

#include <memory>

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include "ocs2_core/integration/Integrator.h"

namespace ocs2 {

class LoopshapingFilterDynamics {
 public:
  explicit LoopshapingFilterDynamics(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : loopshapingDefinition_(std::move(loopshapingDefinition)), integrator_(newIntegrator(IntegratorType::ODE45)) {
    filter_state_.setZero(loopshapingDefinition_->getInputFilter().getNumStates());
  }

  void integrate(scalar_t dt, const vector_t& input);

  void setFilterState(const vector_t& filter_state) { filter_state_ = filter_state; };
  const vector_t& getFilterState() const { return filter_state_; };

 private:
  vector_t computeFlowMap(scalar_t time, const vector_t& filter_state, const vector_t& input) const;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  vector_t filter_state_;
  std::unique_ptr<IntegratorBase> integrator_;
};

}  // namespace ocs2
