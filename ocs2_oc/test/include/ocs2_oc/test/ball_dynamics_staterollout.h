#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>

namespace ocs2 {

class ballLogic final : public HybridLogicRules {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = HybridLogicRules;

  ballLogic() = default;
  ~ballLogic() override = default;

  ballLogic(scalar_array_t switchingTimes, size_array_t subsystemsSequence)
      : BASE(std::move(switchingTimes), std::move(subsystemsSequence)) {}

  void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) override {}

  void update() override {}

 protected:
  void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
                                  const scalar_t& finalTime) override{};
};

class ballDyn : public ControlledSystemBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ControlledSystemBase<2, 1>;
  using state_matrix_t = typename BASE::DIMENSIONS::state_matrix_t;
  using state_input_matrix_t = typename BASE::DIMENSIONS::state_input_matrix_t;

  ballDyn() = default;
  ~ballDyn() = default;

  void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) override {
    state_matrix_t A;
    A << 0.0, 1.0, 0.0, 0.0;
    state_vector_t F;
    F << 0.0, -9.81;

    dxdt = A * x + F;
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    mappedState[0] = state[0];
    mappedState[1] = -0.95 * state[1];
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) override {
    guardSurfacesValue.resize(2);
    guardSurfacesValue[0] = state[0];
    guardSurfacesValue[1] = -state[0] + 0.5;
  }

  ballDyn* clone() const override { return new ballDyn(*this); }
};
}  // namespace ocs2
