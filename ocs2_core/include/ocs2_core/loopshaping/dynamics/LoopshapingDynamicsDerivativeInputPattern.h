//
// Created by ruben on 14.09.18.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/dynamics/LoopshapingDynamicsDerivative.h>

namespace ocs2 {
class LoopshapingDynamicsDerivativeInputPattern final : public LoopshapingDynamicsDerivative {
 public:
  using BASE = LoopshapingDynamicsDerivative;

  LoopshapingDynamicsDerivativeInputPattern(const DerivativesBase& systemDerivative,
                                            std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemDerivative, std::move(loopshapingDefinition)) {}

  ~LoopshapingDynamicsDerivativeInputPattern() override = default;

  LoopshapingDynamicsDerivativeInputPattern(const LoopshapingDynamicsDerivativeInputPattern& obj) = default;

  LoopshapingDynamicsDerivativeInputPattern* clone() const override { return new LoopshapingDynamicsDerivativeInputPattern(*this); };

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
    A.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
    A.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    A.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = s_filter.getA();
    return A;
  }

  matrix_t loopshapingFlowMapDerivativeInput() override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = B_system_.rows();
    const size_t SYSTEM_INPUT_DIM = B_system_.cols();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
    matrix_t B(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_INPUT_DIM + FILTER_INPUT_DIM);
    B.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM) = B_system_;
    B.topRightCorner(SYSTEM_STATE_DIM, FILTER_INPUT_DIM).setZero();
    B.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_INPUT_DIM).setZero();
    B.bottomRightCorner(FILTER_STATE_DIM, FILTER_INPUT_DIM) = s_filter.getB();
    return B;
  }

  matrix_t loopshapingJumpMapDerivativeState() override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = G_system_.rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    matrix_t G(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
    G.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = G_system_;
    G.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
    G.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    G.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM).setIdentity();
    return G;
  }

  matrix_t loopshapingJumpMapDerivativeInput() override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = H_system_.rows();
    const size_t SYSTEM_INPUT_DIM = H_system_.cols();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
    matrix_t H(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_INPUT_DIM + FILTER_INPUT_DIM);
    H.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM) = H_system_;
    H.topRightCorner(SYSTEM_STATE_DIM, FILTER_INPUT_DIM).setZero();
    H.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_INPUT_DIM).setZero();
    H.bottomRightCorner(FILTER_STATE_DIM, FILTER_INPUT_DIM).setZero();
    return H;
  }
};

}  // namespace ocs2
