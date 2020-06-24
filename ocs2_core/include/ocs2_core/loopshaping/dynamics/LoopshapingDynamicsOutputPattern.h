
#pragma once

#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamics.h>

namespace ocs2 {

class LoopshapingDynamicsOutputPattern final : public LoopshapingDynamics {
 public:
  using BASE = LoopshapingDynamics;

  LoopshapingDynamicsOutputPattern(const ControlledSystemBase& controlledSystem,
                                   std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(controlledSystem, std::move(loopshapingDefinition)) {}

  ~LoopshapingDynamicsOutputPattern() override = default;

  LoopshapingDynamicsOutputPattern(const LoopshapingDynamicsOutputPattern& obj) = default;

  LoopshapingDynamicsOutputPattern* clone() const override { return new LoopshapingDynamicsOutputPattern(*this); };

 protected:
  using BASE::loopshapingDefinition_;

 private:
  vector_t filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) override {
    const auto& r_filter = loopshapingDefinition_->getInputFilter();
    return r_filter.getA() * x_filter + r_filter.getB() * u_system;
  }
};

}  // namespace ocs2
