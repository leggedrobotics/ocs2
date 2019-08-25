

#ifndef OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEELIMINATEPATTERN_H
#define OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEELIMINATEPATTERN_H

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingDynamicsDerivativeEliminatePattern final
    : public LoopshapingDynamicsDerivative<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                           FILTER_INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingDynamicsDerivativeEliminatePattern>;

  using BASE =
      LoopshapingDynamicsDerivative<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::system_state_input_matrix_t;
  using typename BASE::system_state_matrix_t;

  using typename BASE::SYSTEM_DERIVATIVE;

  LoopshapingDynamicsDerivativeEliminatePattern(const SYSTEM_DERIVATIVE& systemDerivative,
                                                std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemDerivative, std::move(loopshapingDefinition)) {}

  ~LoopshapingDynamicsDerivativeEliminatePattern() override = default;

  LoopshapingDynamicsDerivativeEliminatePattern(const LoopshapingDynamicsDerivativeEliminatePattern& obj) = default;

  LoopshapingDynamicsDerivativeEliminatePattern* clone() const override {
    return new LoopshapingDynamicsDerivativeEliminatePattern(*this);
  };

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
    A.block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).noalias() = B_system_ * s_filter.getC();
    A.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    A.block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM) = s_filter.getA();
  };

  void loopshapingFlowMapDerivativeInput(state_input_matrix_t& B) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    B.block(0, 0, SYSTEM_STATE_DIM, FILTER_INPUT_DIM).noalias() = B_system_ * s_filter.getD();
    B.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, FILTER_INPUT_DIM) = s_filter.getB();
  };

  void loopshapingJumpMapDerivativeState(state_matrix_t& G) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    G.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = G_system_;
    G.block(0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM).noalias() = H_system_ * s_filter.getC();
    G.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
    G.block(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM, FILTER_STATE_DIM, FILTER_STATE_DIM).setIdentity();
  };

  void loopshapingJumpMapDerivativeInput(state_input_matrix_t& H) override {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    H.block(0, 0, SYSTEM_STATE_DIM, FILTER_INPUT_DIM).noalias() = H_system_ * s_filter.getD();
    H.block(SYSTEM_STATE_DIM, 0, FILTER_STATE_DIM, FILTER_INPUT_DIM).setZero();
  };
};

}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGDYNAMICSDERIVATIVEELIMINATEPATTERN_H
