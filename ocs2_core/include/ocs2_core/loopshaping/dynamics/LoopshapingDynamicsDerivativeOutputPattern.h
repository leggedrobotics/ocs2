//
// Created by rgrandia on 30.04.19.
//

#ifndef OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEOUTPUTPATTERN_H
#define OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEOUTPUTPATTERN_H

namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingDynamicsDerivativeOutputPattern final : public LoopshapingDynamicsDerivative<FULL_STATE_DIM,
                                                                                              FULL_INPUT_DIM,
                                                                                              SYSTEM_STATE_DIM,
                                                                                              SYSTEM_INPUT_DIM,
                                                                                              FILTER_STATE_DIM,
                                                                                              FILTER_INPUT_DIM,
                                                                                              LOGIC_RULES_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingDynamicsDerivativeOutputPattern>;

  using BASE = LoopshapingDynamicsDerivative<FULL_STATE_DIM, FULL_INPUT_DIM,
                                             SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
                                             FILTER_STATE_DIM, FILTER_INPUT_DIM, LOGIC_RULES_T>;
  using typename BASE::state_matrix_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::system_state_matrix_t;
  using typename BASE::system_state_input_matrix_t;

  using typename BASE::SYSTEM_DERIVATIVE;

  LoopshapingDynamicsDerivativeOutputPattern(const SYSTEM_DERIVATIVE &systemDerivative,
                                             std::shared_ptr <LoopshapingDefinition> loopshapingDefinition) :
      BASE(systemDerivative, std::move(loopshapingDefinition)) {}

  ~LoopshapingDynamicsDerivativeOutputPattern() override = default;

  LoopshapingDynamicsDerivativeOutputPattern(const LoopshapingDynamicsDerivativeOutputPattern &obj) = default;

  LoopshapingDynamicsDerivativeOutputPattern* clone() const override {
    return new LoopshapingDynamicsDerivativeOutputPattern(*this);
  };

 protected:
  using BASE::loopshapingDefinition_;

 private:
  void loopshapingFlowMapDerivativeState(const system_state_matrix_t &A_system, state_matrix_t &A) override {
    const auto &r_filter = loopshapingDefinition_->getInputFilter_r();
    A.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = A_system;
    A.block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
    A.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    A.block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM) = r_filter.getA();
  };

  void loopshapingFlowMapDerivativeInput(const system_state_input_matrix_t &B_system,
                                         state_input_matrix_t &B) override {
    const auto &r_filter = loopshapingDefinition_->getInputFilter_r();
    B.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM) = B_system;
    B.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, FILTER_INPUT_DIM) = r_filter.getB();
  };

  void loopshapingJumpMapDerivativeState(const system_state_matrix_t &G_system, state_matrix_t &G) override {
    G.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = G_system;
    G.block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
    G.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    G.block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM).setIdentity();
  };

  void loopshapingJumpMapDerivativeInput(const system_state_input_matrix_t &H_system,
                                         state_input_matrix_t &H) override {
    H.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM) = H_system;
    H.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_INPUT_DIM).setZero();
  };
};
}

#endif //OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEOUTPUTPATTERN_H
