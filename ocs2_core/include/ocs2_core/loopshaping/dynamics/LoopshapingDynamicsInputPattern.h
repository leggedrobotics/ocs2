//
// Created by ruben on 14.09.18.
//

#pragma once

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h>

namespace ocs2 {

class LoopshapingDynamicsInputPattern final : public LoopshapingDynamics {
 public:
  using BASE = LoopshapingDynamics;

  LoopshapingDynamicsInputPattern(const SystemDynamicsBase& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                                  std::shared_ptr<LoopshapingPreComputation> preCompPtr)
      : BASE(controlledSystem, std::move(loopshapingDefinition), std::move(preCompPtr)) {}

  ~LoopshapingDynamicsInputPattern() override = default;

  LoopshapingDynamicsInputPattern(const LoopshapingDynamicsInputPattern& obj) = delete;

  LoopshapingDynamicsInputPattern* clone(std::shared_ptr<PreComputation> preCompPtr) const override {
    return new LoopshapingDynamicsInputPattern(*systemDynamics_, loopshapingDefinition_,
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
