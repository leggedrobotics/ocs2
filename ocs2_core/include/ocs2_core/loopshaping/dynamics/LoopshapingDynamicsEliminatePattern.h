
#pragma once

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h>

namespace ocs2 {

class LoopshapingDynamicsEliminatePattern final : public LoopshapingDynamics {
 public:
  using BASE = LoopshapingDynamics;

  LoopshapingDynamicsEliminatePattern(const SystemDynamicsBase& controlledSystem,
                                      std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                                      std::unique_ptr<LoopshapingPreComputation> preCompPtr)
      : BASE(controlledSystem, std::move(loopshapingDefinition), std::move(preCompPtr)) {}

  ~LoopshapingDynamicsEliminatePattern() override = default;

  LoopshapingDynamicsEliminatePattern(const LoopshapingDynamicsEliminatePattern& obj) = default;

  LoopshapingDynamicsEliminatePattern* clone() const override { return new LoopshapingDynamicsEliminatePattern(*this); }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation* preCompPtr) override;

 private:
  vector_t filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) override;

  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
