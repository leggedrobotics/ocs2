
#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsDerivative.h>

namespace ocs2 {

class LoopshapingDynamicsDerivativeOutputPattern final : public LoopshapingDynamicsDerivative {
 public:
  using BASE = LoopshapingDynamicsDerivative;

  LoopshapingDynamicsDerivativeOutputPattern(const DerivativesBase& systemDerivative,
                                             std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemDerivative, std::move(loopshapingDefinition)) {}

  ~LoopshapingDynamicsDerivativeOutputPattern() override = default;

  LoopshapingDynamicsDerivativeOutputPattern(const LoopshapingDynamicsDerivativeOutputPattern& obj) = default;

  LoopshapingDynamicsDerivativeOutputPattern* clone() const override { return new LoopshapingDynamicsDerivativeOutputPattern(*this); };

 protected:
  using BASE::A_system_;
  using BASE::B_system_;
  using BASE::G_system_;
  using BASE::H_system_;
  using BASE::loopshapingDefinition_;

 private:
  matrix_t loopshapingFlowMapDerivativeState() override {
    const auto& r_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = A_system_.rows();
    const size_t FILTER_STATE_DIM = r_filter.getNumStates();
    matrix_t A(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
    A.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = A_system_;
    A.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
    A.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    A.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = r_filter.getA();
    return A;
  }

  matrix_t loopshapingFlowMapDerivativeInput() override {
    const auto& r_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = B_system_.rows();
    const size_t SYSTEM_INPUT_DIM = B_system_.cols();
    const size_t FILTER_STATE_DIM = r_filter.getNumStates();
    matrix_t B(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_INPUT_DIM);
    B.topRows(SYSTEM_STATE_DIM) = B_system_;
    B.bottomRows(FILTER_STATE_DIM) = r_filter.getB();
    return B;
  }

  matrix_t loopshapingJumpMapDerivativeState() override {
    const auto& r_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = G_system_.rows();
    const size_t FILTER_STATE_DIM = r_filter.getNumStates();
    matrix_t G(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
    G.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = G_system_;
    G.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
    G.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    G.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM).setIdentity();
    return G;
  }

  matrix_t loopshapingJumpMapDerivativeInput() override {
    const auto& r_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = H_system_.rows();
    const size_t SYSTEM_INPUT_DIM = H_system_.cols();
    const size_t FILTER_STATE_DIM = r_filter.getNumStates();
    matrix_t H(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_INPUT_DIM);
    H.topRows(SYSTEM_STATE_DIM) = H_system_;
    H.bottomRows(FILTER_STATE_DIM).setZero();
    return H;
  }
};

}  // namespace ocs2
