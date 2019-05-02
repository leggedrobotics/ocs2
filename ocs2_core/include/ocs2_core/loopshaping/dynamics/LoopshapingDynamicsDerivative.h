//
// Created by ruben on 14.09.18.
//

#ifndef OCS2_LOOPSHAPINGDYNAMICSDERIVATIVE_H
#define OCS2_LOOPSHAPINGDYNAMICSDERIVATIVE_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/dynamics/DerivativesBase.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/logic/machine/LogicRulesMachine.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingDynamicsDerivative : public DerivativesBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingDynamicsDerivative>;

  using BASE = DerivativesBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>;
  using scalar_t = typename BASE::scalar_t;
  using state_vector_t = typename BASE::state_vector_t;
  using input_vector_t = typename BASE::input_vector_t;
  using state_matrix_t = typename BASE::state_matrix_t;
  using state_input_matrix_t = typename BASE::state_input_matrix_t;
  using dynamic_vector_t = typename BASE::dynamic_vector_t;
  using dynamic_state_matrix_t = typename BASE::dynamic_state_matrix_t;
  using dynamic_input_matrix_t = typename BASE::dynamic_input_matrix_t;

  using SYSTEM_DERIVATIVE = DerivativesBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;
  static constexpr size_t system_state_dim = SYSTEM_STATE_DIM;
  static constexpr size_t system_input_dim = SYSTEM_INPUT_DIM;
  using system_state_vector_t = typename SYSTEM_DERIVATIVE::state_vector_t;
  using system_input_vector_t = typename SYSTEM_DERIVATIVE::input_vector_t;
  using system_state_matrix_t = typename SYSTEM_DERIVATIVE::state_matrix_t;
  using system_state_input_matrix_t = typename SYSTEM_DERIVATIVE::state_input_matrix_t;
  using system_dynamic_state_matrix_t = typename SYSTEM_DERIVATIVE::dynamic_state_matrix_t;
  using system_dynamic_input_matrix_t = typename SYSTEM_DERIVATIVE::dynamic_input_matrix_t;

  static constexpr size_t filter_state_dim = FILTER_STATE_DIM;
  static constexpr size_t filter_input_dim = FILTER_INPUT_DIM;
  using filter_state_vector_t = Eigen::Matrix<scalar_t, filter_state_dim, 1>;
  using filter_input_vector_t = Eigen::Matrix<scalar_t, filter_input_dim, 1>;

  ~LoopshapingDynamicsDerivative() override = default;

  LoopshapingDynamicsDerivative(const LoopshapingDynamicsDerivative &obj) :
      BASE(),
      systemDerivative_(obj.systemDerivative_->clone()),
      loopshapingDefinition_(obj.loopshapingDefinition_) {}

  void initializeModel(
      LogicRulesMachine<LOGIC_RULES_T> &logicRulesMachine,
      const size_t &partitionIndex,
      const char *algorithmName = NULL) override {
    BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    systemDerivative_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
  }

  static std::unique_ptr<LoopshapingDynamicsDerivative> Create(const SYSTEM_DERIVATIVE &controlledSystem,
                                                               std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void setCurrentStateAndControl(
      const scalar_t &t,
      const state_vector_t &x,
      const input_vector_t &u) override {
    BASE::setCurrentStateAndControl(t, x, u);

    system_state_vector_t systemstate;
    system_input_vector_t systeminput;
    loopshapingDefinition_->getSystemState(x, systemstate);
    loopshapingDefinition_->getSystemInput(x, u, systeminput);
    systemDerivative_->setCurrentStateAndControl(t, systemstate, systeminput);
  }

  void getFlowMapDerivativeTime(state_vector_t &df) override {
    system_state_vector_t system_df;
    filter_state_vector_t filter_df;
    systemDerivative_->getFlowMapDerivativeTime(system_df);
    filter_df.setZero();
    loopshapingDefinition_->concatenateSystemAndFilterState(system_df, filter_df, df);
  }

  void getFlowMapDerivativeState(state_matrix_t &A) override {
    // system
    system_state_matrix_t A_system;
    systemDerivative_->getFlowMapDerivativeState(A_system);

    loopshapingFlowMapDerivativeState(A_system, A);
  };

  void getFlowMapDerivativeInput(state_input_matrix_t &B) override {
    // system
    system_state_input_matrix_t B_system;
    systemDerivative_->getFlowMapDerivativeInput(B_system);

    loopshapingFlowMapDerivativeInput(B_system, B);
  };

  void getJumpMapDerivativeTime(state_vector_t &dg) {
    system_state_vector_t system_dg;
    filter_state_vector_t filter_dg;
    systemDerivative_->getJumpMapDerivativeTime(system_dg);
    filter_dg.setZero();
    loopshapingDefinition_->concatenateSystemAndFilterState(system_dg, filter_dg, dg);
  }

  void getJumpMapDerivativeState(state_matrix_t &G) override {
    // system
    system_state_matrix_t G_system;
    systemDerivative_->getJumpMapDerivativeState(G_system);

    loopshapingJumpMapDerivativeState(G_system, G);
  }

  void getJumpMapDerivativeInput(state_input_matrix_t &H) override {
    // system
    system_state_input_matrix_t H_system;
    systemDerivative_->getJumpMapDerivativeInput(H_system);

    loopshapingJumpMapDerivativeInput(H_system, H);
  }

  void getGuardSurfacesDerivativeTime(dynamic_vector_t &D_t_gamma) override {
    throw std::runtime_error("[LoopshapingDynamicsDerivative] Guard surfaces not implemented");
  }

  void getGuardSurfacesDerivativeState(dynamic_state_matrix_t &D_x_gamma) override {
    throw std::runtime_error("[LoopshapingDynamicsDerivative] Guard surfaces not implemented");
  }

  void getGuardSurfacesDerivativeInput(dynamic_input_matrix_t &D_u_gamma) override {
    throw std::runtime_error("[LoopshapingDynamicsDerivative] Guard surfaces not implemented");
  }

 protected:
  LoopshapingDynamicsDerivative(const SYSTEM_DERIVATIVE &systemDerivative,
                                std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) :
      BASE(),
      systemDerivative_(systemDerivative.clone()),
      loopshapingDefinition_(std::move(loopshapingDefinition)) {};

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

 private:
  std::unique_ptr<SYSTEM_DERIVATIVE> systemDerivative_;
  virtual void loopshapingFlowMapDerivativeState(const system_state_matrix_t &A_system, state_matrix_t &A) = 0;
  virtual void loopshapingFlowMapDerivativeInput(const system_state_input_matrix_t &B_system,
                                                 state_input_matrix_t &B) = 0;
  virtual void loopshapingJumpMapDerivativeState(const system_state_matrix_t &G_system, state_matrix_t &G) = 0;
  virtual void loopshapingJumpMapDerivativeInput(const system_state_input_matrix_t &H_system,
                                                 state_input_matrix_t &H) = 0;
};

} // namespace ocs2

#include "LoopshapingDynamicsDerivativeInputPattern.h"
#include "LoopshapingDynamicsDerivativeOutputPattern.h"

// Implement Factory method
namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T>
std::unique_ptr<LoopshapingDynamicsDerivative<FULL_STATE_DIM,
                                              FULL_INPUT_DIM,
                                              SYSTEM_STATE_DIM,
                                              SYSTEM_INPUT_DIM,
                                              FILTER_STATE_DIM,
                                              FILTER_INPUT_DIM,
                                              LOGIC_RULES_T>> LoopshapingDynamicsDerivative<FULL_STATE_DIM,
                                                                                            FULL_INPUT_DIM,
                                                                                            SYSTEM_STATE_DIM,
                                                                                            SYSTEM_INPUT_DIM,
                                                                                            FILTER_STATE_DIM,
                                                                                            FILTER_INPUT_DIM,
                                                                                            LOGIC_RULES_T>::Create(
    const SYSTEM_DERIVATIVE &controlledSystem,
    std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  if (loopshapingDefinition->getInputFilter_s().getNumOutputs() > 0) {
    return std::unique_ptr<LoopshapingDynamicsDerivative>(new LoopshapingDynamicsDerivativeInputPattern<FULL_STATE_DIM,
                                                                                                        FULL_INPUT_DIM,
                                                                                                        SYSTEM_STATE_DIM,
                                                                                                        SYSTEM_INPUT_DIM,
                                                                                                        FILTER_STATE_DIM,
                                                                                                        FILTER_INPUT_DIM,
                                                                                                        LOGIC_RULES_T>(
        controlledSystem,
        std::move(
            loopshapingDefinition)));
  } else if (loopshapingDefinition->getInputFilter_r().getNumOutputs() > 0) {
    return std::unique_ptr<LoopshapingDynamicsDerivative>(new LoopshapingDynamicsDerivativeOutputPattern<FULL_STATE_DIM,
                                                                                                         FULL_INPUT_DIM,
                                                                                                         SYSTEM_STATE_DIM,
                                                                                                         SYSTEM_INPUT_DIM,
                                                                                                         FILTER_STATE_DIM,
                                                                                                         FILTER_INPUT_DIM,
                                                                                                         LOGIC_RULES_T>(
        controlledSystem,
        std::move(
            loopshapingDefinition)));
  }
};
}; // namespace ocs2

#endif //OCS2_LOOPSHAPINGDYNAMICSDERIVATIVE_H
