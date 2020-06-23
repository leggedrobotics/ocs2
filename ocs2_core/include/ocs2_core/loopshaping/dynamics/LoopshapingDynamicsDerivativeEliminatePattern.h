
#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsDerivative.h>

namespace ocs2 {

class LoopshapingDynamicsDerivativeEliminatePattern final : public LoopshapingDynamicsDerivative {
 public:
  using BASE = LoopshapingDynamicsDerivative;

  LoopshapingDynamicsDerivativeEliminatePattern(const DerivativesBase& systemDerivative,
                                                std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemDerivative, std::move(loopshapingDefinition)) {}

  LoopshapingDynamicsDerivativeEliminatePattern(const LoopshapingDynamicsDerivativeEliminatePattern& obj) = default;

  ~LoopshapingDynamicsDerivativeEliminatePattern() override = default;

  LoopshapingDynamicsDerivativeEliminatePattern* clone() const override { return new LoopshapingDynamicsDerivativeEliminatePattern(*this); }

 protected:
  using BASE::A_system_;
  using BASE::B_system_;
  using BASE::G_system_;
  using BASE::H_system_;
  using BASE::loopshapingDefinition_;

 private:
  matrix_t loopshapingFlowMapDerivativeState() override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = A_system_.rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    matrix_t A(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
    A.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = A_system_;
    A.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).noalias() = B_system_ * s_filter.getC();
    A.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    A.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = s_filter.getA();
    return A;
  }

  matrix_t loopshapingFlowMapDerivativeInput() override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = B_system_.rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
    matrix_t B(SYSTEM_STATE_DIM + FILTER_STATE_DIM, FILTER_INPUT_DIM);
    B.topRows(SYSTEM_STATE_DIM).noalias() = B_system_ * s_filter.getD();
    B.bottomRows(FILTER_STATE_DIM) = s_filter.getB();
    return B;
  }

  matrix_t loopshapingJumpMapDerivativeState() override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = G_system_.rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    matrix_t G(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
    G.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = G_system_;
    G.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).noalias() = H_system_ * s_filter.getC();
    G.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    G.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM).setIdentity();
    return G;
  }

  matrix_t loopshapingJumpMapDerivativeInput() override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = H_system_.rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
    matrix_t H(SYSTEM_STATE_DIM + FILTER_STATE_DIM, FILTER_INPUT_DIM);
    H.topRows(SYSTEM_STATE_DIM).noalias() = H_system_ * s_filter.getD();
    H.bottomRows(FILTER_STATE_DIM).setZero();
    return H;
  }
};

}  // namespace ocs2
