//
// Created by ruben on 14.09.18.
//

#ifndef OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEINPUTPATTERN_H
#define OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEINPUTPATTERN_H

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingDynamicsDerivativeInputPattern final
    : public LoopshapingDynamicsDerivative<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                           FILTER_INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingDynamicsDerivativeInputPattern>;

  using BASE =
      LoopshapingDynamicsDerivative<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::system_state_input_matrix_t;
  using typename BASE::system_state_matrix_t;

  using typename BASE::SYSTEM_DERIVATIVE;

  LoopshapingDynamicsDerivativeInputPattern(const SYSTEM_DERIVATIVE& systemDerivative,
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
  void loopshapingFlowMapDerivativeState(state_matrix_t& A) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    A.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = A_system_;
    A.block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
    A.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    A.block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM) = s_filter.getA();
  };

  void loopshapingFlowMapDerivativeInput(state_input_matrix_t& B) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    B.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM) = B_system_;
    B.block(0, SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM, FILTER_INPUT_DIM).setZero();
    B.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_INPUT_DIM).setZero();
    B.block(SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM) = s_filter.getB();
  };

  void loopshapingJumpMapDerivativeState(state_matrix_t& G) override {
    G.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = G_system_;
    G.block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
    G.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    G.block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM).setIdentity();
  };

  void loopshapingJumpMapDerivativeInput(state_input_matrix_t& H) override {
    H.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM) = H_system_;
    H.block(0, SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM, FILTER_INPUT_DIM).setZero();
    H.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_INPUT_DIM).setZero();
    H.block(SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM).setZero();
  };
};

}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEINPUTPATTERN_H
