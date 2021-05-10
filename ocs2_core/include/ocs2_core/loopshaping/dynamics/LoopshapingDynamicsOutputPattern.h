
#pragma once

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h>

namespace ocs2 {

class LoopshapingDynamicsOutputPattern final : public LoopshapingDynamics {
 public:
  using BASE = LoopshapingDynamics;

  LoopshapingDynamicsOutputPattern(const SystemDynamicsBase& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                                   std::shared_ptr<LoopshapingPreComputation> preCompPtr)
      : BASE(controlledSystem, std::move(loopshapingDefinition), std::move(preCompPtr)) {}

  ~LoopshapingDynamicsOutputPattern() override = default;

  LoopshapingDynamicsOutputPattern(const LoopshapingDynamicsOutputPattern& obj) = delete;

  LoopshapingDynamicsOutputPattern* clone(std::shared_ptr<PreComputation> preCompPtr) const override {
    return new LoopshapingDynamicsOutputPattern(*systemDynamics_, loopshapingDefinition_,
                                                std::dynamic_pointer_cast<LoopshapingPreComputation>(std::move(preCompPtr)));
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation* preCompPtr) override;

 protected:
  using BASE::loopshapingDefinition_;

 private:
  vector_t filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) override;
};

}  // namespace ocs2
