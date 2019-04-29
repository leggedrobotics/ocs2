//
// Created by rgrandia on 29.04.19.
//


#ifndef OCS2_LOOPSHAPINGCOSTINPUTPATTERN_H
#define OCS2_LOOPSHAPINGCOSTINPUTPATTERN_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "LoopshapingCostImplementationBase.h"

namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingCostInputPattern final : public LoopshapingCostImplementationBase<FULL_STATE_DIM, FULL_INPUT_DIM,
                                                                                   SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM,
                                                                                   FILTER_STATE_DIM, FILTER_INPUT_DIM, NullLogicRules> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using FULL_DIMENSIONS = ocs2::Dimensions<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using scalar_t = typename FULL_DIMENSIONS::scalar_t;
  using state_vector_t = typename FULL_DIMENSIONS::state_vector_t;
  using input_vector_t = typename FULL_DIMENSIONS::input_vector_t;
  using state_matrix_t = typename FULL_DIMENSIONS::state_matrix_t;
  using input_matrix_t = typename FULL_DIMENSIONS::input_matrix_t;
  using input_state_matrix_t = typename FULL_DIMENSIONS::input_state_matrix_t;

  using SYSTEM_DIMENSIONS = ocs2::Dimensions<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEM_DIMENSIONS::state_vector_t;
  using system_input_vector_t = typename SYSTEM_DIMENSIONS::input_vector_t;
  using system_state_matrix_t = typename SYSTEM_DIMENSIONS::state_matrix_t;
  using system_input_matrix_t = typename SYSTEM_DIMENSIONS::input_matrix_t;
  using system_input_state_matrix_t = typename SYSTEM_DIMENSIONS::input_state_matrix_t;

  using FILTER_DIMENSIONS = ocs2::Dimensions<FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using filter_state_vector_t = typename FILTER_DIMENSIONS::state_vector_t;
  using filter_input_vector_t = typename FILTER_DIMENSIONS::input_vector_t;

  using SYSTEMCOST = CostFunctionBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, LOGIC_RULES_T>;

  LoopshapingCostInputPattern(std::shared_ptr <SYSTEMCOST> systemCost,
                              std::shared_ptr <LoopshapingDefinition> loopshapingDefinition
  )
      :
      systemCost_(systemCost),
      loopshapingDefinition_(loopshapingDefinition) {
  }

  ~LoopshapingCostInputPattern() override = default;

  void setCurrentStateAndControl(
      const scalar_t &t,
      const system_state_vector_t &x_system,
      const system_input_vector_t &u_system,
      const filter_state_vector_t &x_filter,
      const filter_input_vector_t &u_filter) {

    // Compute cost and approximation around system input
    systemCost_->setCurrentStateAndControl(t, x_system, u_system);
    systemCost_->getIntermediateCostSecondDerivativeState(Q_system_);
    systemCost_->getIntermediateCostDerivativeInputState(P_system_);
    systemCost_->getIntermediateCostSecondDerivativeInput(R_system_);
    systemCost_->getIntermediateCostDerivativeState(q_system_);
    systemCost_->getIntermediateCostDerivativeInput(r_system_);
    systemCost_->getIntermediateCost(c_system_);

    // Compute cost and approximation around filter input
    systemCost_->setCurrentStateAndControl(t, x_system, u_filter);
    systemCost_->getIntermediateCostSecondDerivativeState(Q_filter_);
    systemCost_->getIntermediateCostDerivativeInputState(P_filter_);
    systemCost_->getIntermediateCostSecondDerivativeInput(R_filter_);
    systemCost_->getIntermediateCostDerivativeState(q_filter_);
    systemCost_->getIntermediateCostDerivativeInput(r_filter_);
    systemCost_->getIntermediateCost(c_filter_);
  }

  void getIntermediateCost(scalar_t &L) override {
    const auto &gamma = loopshapingDefinition_->gamma;
    L = gamma * c_filter_ + (1.0 - gamma) * c_system_;
  };

  void getIntermediateCostDerivativeTime(scalar_t &dLdt) override {
    // TODO
    dLdt = 0;
  }

  void getIntermediateCostDerivativeState(state_vector_t &dLdx) override {
    const auto &gamma = loopshapingDefinition_->gamma;
    dLdx.segment(0, SYSTEM_STATE_DIM) = gamma * q_filter_ + (1.0 - gamma) * q_system_;
    dLdx.segment(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
  };

  void getIntermediateCostSecondDerivativeState(state_matrix_t &dLdxx) override {
    auto &gamma = loopshapingDefinition_->gamma;
    dLdxx.setZero();
    dLdxx.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = gamma * Q_filter_ + (1.0 - gamma) * Q_system_;
  };

  void getIntermediateCostDerivativeInput(input_vector_t &dLdu) override {
    const auto &gamma = loopshapingDefinition_->gamma;
    dLdu.segment(0, SYSTEM_INPUT_DIM) = (1.0 - gamma) * r_system_;
    dLdu.segment(SYSTEM_INPUT_DIM, FILTER_INPUT_DIM) = gamma * r_filter_;

  };

  void getIntermediateCostSecondDerivativeInput(input_matrix_t &dLduu) override {
    const auto &gamma = loopshapingDefinition_->gamma;
    dLduu.setZero();
    dLduu.block(0, 0, SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) = (1.0 - gamma) * R_system_;
    dLduu.block(SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM, FILTER_INPUT_DIM, FILTER_INPUT_DIM) = gamma * R_filter_;

  };

  void getIntermediateCostDerivativeInputState(input_state_matrix_t &dLdux) override {
    const auto &gamma = loopshapingDefinition_->gamma;
    dLdux.block(0, 0, SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM) = (1.0 - gamma) * P_system_;
    dLdux.block(0, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM).setZero();
    dLdux.block(SYSTEM_INPUT_DIM, 0, FILTER_INPUT_DIM, SYSTEM_STATE_DIM) = gamma * P_filter_;
    dLdux.block(SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM, FILTER_INPUT_DIM, FILTER_STATE_DIM).setZero();
  };

 private:
  system_state_matrix_t Q_system_;
  system_input_matrix_t R_system_;
  system_input_state_matrix_t P_system_;
  system_state_vector_t q_system_;
  system_input_vector_t r_system_;
  scalar_t c_system_;

  system_state_matrix_t Q_filter_;
  system_input_matrix_t R_filter_;
  system_input_state_matrix_t P_filter_;
  system_state_vector_t q_filter_;
  system_input_vector_t r_filter_;
  scalar_t c_filter_;

  std::shared_ptr <SYSTEMCOST> systemCost_;
  std::shared_ptr <LoopshapingDefinition> loopshapingDefinition_;
};
}; // namespace ocs2

#endif //OCS2_LOOPSHAPINGCOSTINPUTPATTERN_H
