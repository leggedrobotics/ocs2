
#pragma once

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h>

namespace ocs2 {

class LoopshapingDynamicsOutputPattern final : public LoopshapingDynamics {
 public:
  using BASE = LoopshapingDynamics;

  LoopshapingDynamicsOutputPattern(const SystemDynamicsBase& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                                   const LoopshapingPreComputation& PreComputation)
      : BASE(controlledSystem, std::move(loopshapingDefinition), PreComputation) {}

  ~LoopshapingDynamicsOutputPattern() override = default;

  LoopshapingDynamicsOutputPattern(const LoopshapingDynamicsOutputPattern& obj) = default;

  LoopshapingDynamicsOutputPattern* clone() const override { return new LoopshapingDynamicsOutputPattern(*this); }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override;

 private:
  vector_t filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) override;

  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
