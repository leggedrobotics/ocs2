//
// Created by ruben on 14.09.18.
//

#pragma once

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h>

namespace ocs2 {

class LoopshapingDynamicsInputPattern final : public LoopshapingDynamics {
 public:
  using BASE = LoopshapingDynamics;

  LoopshapingDynamicsInputPattern(const ControlledSystemBase& controlledSystem,
                                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(controlledSystem, std::move(loopshapingDefinition)) {}

  ~LoopshapingDynamicsInputPattern() override = default;

  LoopshapingDynamicsInputPattern(const LoopshapingDynamicsInputPattern& obj) = default;

  LoopshapingDynamicsInputPattern* clone() const override { return new LoopshapingDynamicsInputPattern(*this); };

 protected:
  using BASE::loopshapingDefinition_;

 private:
  vector_t filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    vector_t filterStateDerivative;
    filterStateDerivative.noalias() = s_filter.getA() * x_filter;
    filterStateDerivative.noalias() += s_filter.getB() * u_filter;
    return filterStateDerivative;
  }
};

}  // namespace ocs2
