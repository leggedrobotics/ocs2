//
// Created by ruben on 14.09.18.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {
class LoopshapingDynamics : public ControlledSystemBase {
 public:
  ~LoopshapingDynamics() override = default;

  static std::unique_ptr<LoopshapingDynamics> create(const ControlledSystemBase& controlledSystem,
                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input) override;
  vector_t computeJumpMap(scalar_t time, const vector_t& state) override;
  vector_t computeGuardSurfaces(scalar_t time, const vector_t& state) override;

 protected:
  LoopshapingDynamics(const LoopshapingDynamics& obj)
      : controlledSystem_(obj.controlledSystem_->clone()), loopshapingDefinition_(obj.loopshapingDefinition_) {}

  LoopshapingDynamics(const ControlledSystemBase& controlledSystem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : controlledSystem_(controlledSystem.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

 private:
  std::unique_ptr<ControlledSystemBase> controlledSystem_;

  virtual vector_t filterFlowmap(const vector_t& x_filter, const vector_t& u_filter, const vector_t& u_system) = 0;
};

}  // namespace ocs2
