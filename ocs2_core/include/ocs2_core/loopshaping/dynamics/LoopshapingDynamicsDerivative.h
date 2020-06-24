//
// Created by ruben on 14.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {
class LoopshapingDynamicsDerivative : public DerivativesBase {
 public:
  using BASE = DerivativesBase;

  ~LoopshapingDynamicsDerivative() override = default;

  static std::unique_ptr<LoopshapingDynamicsDerivative> create(const DerivativesBase& controlledSystem,
                                                               std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) override;
  vector_t getFlowMapDerivativeTime() override;
  matrix_t getFlowMapDerivativeState() override;
  matrix_t getFlowMapDerivativeInput() override;
  vector_t getJumpMapDerivativeTime() override;

  matrix_t getJumpMapDerivativeState() override;
  matrix_t getJumpMapDerivativeInput() override;
  vector_t getGuardSurfacesDerivativeTime() override;
  matrix_t getGuardSurfacesDerivativeState() override;
  matrix_t getGuardSurfacesDerivativeInput() override;

 protected:
  LoopshapingDynamicsDerivative(const LoopshapingDynamicsDerivative& obj)
      : BASE(),
        systemDerivative_(obj.systemDerivative_->clone()),
        loopshapingDefinition_(obj.loopshapingDefinition_),
        systemApproximationValid_(false),
        jumpMapApproximationValid_(false) {}

  LoopshapingDynamicsDerivative(const DerivativesBase& systemDerivative, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(),
        systemDerivative_(systemDerivative.clone()),
        loopshapingDefinition_(std::move(loopshapingDefinition)),
        systemApproximationValid_(false),
        jumpMapApproximationValid_(false){};

 protected:
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  matrix_t B_system_;
  matrix_t A_system_;
  matrix_t H_system_;
  matrix_t G_system_;

 private:
  std::unique_ptr<DerivativesBase> systemDerivative_;

  void computeSystemDerivatives();
  bool systemApproximationValid_;
  void computeJumpMapDerivatives();
  bool jumpMapApproximationValid_;

  virtual matrix_t loopshapingFlowMapDerivativeState() = 0;
  virtual matrix_t loopshapingFlowMapDerivativeInput() = 0;
  virtual matrix_t loopshapingJumpMapDerivativeState() = 0;
  virtual matrix_t loopshapingJumpMapDerivativeInput() = 0;
};

}  // namespace ocs2
