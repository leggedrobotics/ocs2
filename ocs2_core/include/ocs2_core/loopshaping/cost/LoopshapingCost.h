//
// Created by ruben on 24.09.18.
//

#ifndef OCS2_LOOPSHAPINGCOST_H
#define OCS2_LOOPSHAPINGCOST_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/cost/QuadraticCostFunction.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"
#include "LoopshapingCostImplementationBase.h"
#include "LoopshapingCostInputPattern.h"
#include "LoopshapingCostOutputPattern.h"

namespace ocs2 {

template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingCost final : public CostFunctionBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingCost>;
  using ConstPtr = std::shared_ptr<const LoopshapingCost>;

  using BASE = CostFunctionBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T>;
  using cost_desired_trajectories_t = typename BASE::cost_desired_trajectories_t;

  using FULL_DIMENSIONS = ocs2::Dimensions<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using scalar_t = typename FULL_DIMENSIONS::scalar_t;
  using state_vector_t = typename FULL_DIMENSIONS::state_vector_t;
  using input_vector_t = typename FULL_DIMENSIONS::input_vector_t;
  using dynamic_vector_t = typename FULL_DIMENSIONS::dynamic_vector_t;
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

  using LoopshapingCostImplementation = LoopshapingCostImplementationBase<FULL_STATE_DIM,
                                                                          FULL_INPUT_DIM,
                                                                          SYSTEM_STATE_DIM,
                                                                          SYSTEM_INPUT_DIM,
                                                                          FILTER_STATE_DIM,
                                                                          FILTER_INPUT_DIM,
                                                                          NullLogicRules>;

  LoopshapingCost(const SYSTEMCOST &systemCost,
                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(),
        systemCost_(systemCost.clone()),
        loopshapingDefinition_(loopshapingDefinition) {

    if (loopshapingDefinition_->getInputFilter_s().getNumOutputs() > 0) {
      loopshapingCostImplementation_.reset(new LoopshapingCostInputPattern<FULL_STATE_DIM,
                                                                           FULL_INPUT_DIM,
                                                                           SYSTEM_STATE_DIM,
                                                                           SYSTEM_INPUT_DIM,
                                                                           FILTER_STATE_DIM,
                                                                           FILTER_INPUT_DIM,
                                                                           NullLogicRules>(systemCost_,
                                                                                           loopshapingDefinition_));
    }
    if (loopshapingDefinition_->getInputFilter_r().getNumOutputs() > 0) {
      loopshapingCostImplementation_.reset(new LoopshapingCostOutputPattern<FULL_STATE_DIM,
                                                                           FULL_INPUT_DIM,
                                                                           SYSTEM_STATE_DIM,
                                                                           SYSTEM_INPUT_DIM,
                                                                           FILTER_STATE_DIM,
                                                                           FILTER_INPUT_DIM,
                                                                           NullLogicRules>(systemCost_,
                                                                                           loopshapingDefinition_));
    }
  }

  ~LoopshapingCost() override = default;

  LoopshapingCost(const LoopshapingCost &obj) :
      BASE(),
      systemCost_(obj.systemCost_->clone()),
      loopshapingDefinition_(obj.loopshapingDefinition_) {
  }

  LoopshapingCost *clone() const override {
    return new LoopshapingCost(*this);
  };

  void initializeModel(
      LogicRulesMachine<LOGIC_RULES_T> &logicRulesMachine,
      const size_t &partitionIndex,
      const char *algorithmName = NULL) override {
    BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    systemCost_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
  }

  void setCurrentStateAndControl(
      const scalar_t &t,
      const state_vector_t &x,
      const input_vector_t &u) override {

    dynamic_vector_t xNominal;
    xNominalFunc_.interpolate(t, xNominal);
    dynamic_vector_t uNominal;
    uNominalFunc_.interpolate(t, uNominal);

    filter_state_vector_t x_filter_;
    filter_input_vector_t u_filter_;
    system_state_vector_t x_system_;
    system_input_vector_t u_system_;
    loopshapingDefinition_->getSystemState(x, x_system_);
    loopshapingDefinition_->getSystemInput(x, u, u_system_);
    loopshapingDefinition_->getFilterState(x, x_filter_);
    loopshapingDefinition_->getFilteredInput(x, u, u_filter_);

    loopshapingCostImplementation_->setCurrentStateAndControl(t, x_system_, u_system_, x_filter_, u_filter_);

    BASE::setCurrentStateAndControl(t, x, u);
  }

  void setCostDesiredTrajectories(const cost_desired_trajectories_t &costDesiredTrajectories) override {
    costDesiredTrajectories.getDesiredStateFunc(xNominalFunc_);
    costDesiredTrajectories.getDesiredInputFunc(uNominalFunc_);

    // Desired trajectories are dynamic size -> must resize for future cast to fixed size vectors
    size_t reference_length = costDesiredTrajectories.desiredTimeTrajectory().size();
    systemCostDesiredTrajectories_ = cost_desired_trajectories_t(reference_length);
    systemCostDesiredTrajectories_.desiredTimeTrajectory() = costDesiredTrajectories.desiredTimeTrajectory();
    auto &systemStateTrajectory = systemCostDesiredTrajectories_.desiredStateTrajectory();
    auto &systemInputTrajectory = systemCostDesiredTrajectories_.desiredInputTrajectory();
    auto &stateTrajectory = costDesiredTrajectories.desiredStateTrajectory();
    auto &inputTrajectory = costDesiredTrajectories.desiredInputTrajectory();
    for (int k = 0; k < reference_length; k++) {
      // For now assume that cost DesiredTrajectory is specified w.r.t original system x, u
      systemStateTrajectory[k] = stateTrajectory[k].segment(0, SYSTEM_STATE_DIM);
      systemInputTrajectory[k] = inputTrajectory[k].segment(0, SYSTEM_INPUT_DIM);
    }

    systemCost_->setCostDesiredTrajectories(systemCostDesiredTrajectories_);
  }

  void getIntermediateCost(scalar_t &L) override {
    loopshapingCostImplementation_->getIntermediateCost(L);
  };

  void getIntermediateCostDerivativeTime(scalar_t &dLdt) override {
    loopshapingCostImplementation_->getIntermediateCostDerivativeTime(dLdt);
  }

  void getIntermediateCostDerivativeState(state_vector_t &dLdx) override {
    loopshapingCostImplementation_->getIntermediateCostDerivativeState(dLdx);
  };

  void getIntermediateCostSecondDerivativeState(state_matrix_t &dLdxx) override {
    loopshapingCostImplementation_->getIntermediateCostSecondDerivativeState(dLdxx);
  };

  void getIntermediateCostDerivativeInput(input_vector_t &dLdu) override {
    loopshapingCostImplementation_->getIntermediateCostDerivativeInput(dLdu);
  };

  void getIntermediateCostSecondDerivativeInput(input_matrix_t &dLduu) override {
    loopshapingCostImplementation_->getIntermediateCostSecondDerivativeInput(dLduu);
  };

  void getIntermediateCostDerivativeInputState(input_state_matrix_t &dLdux) override {
    loopshapingCostImplementation_->getIntermediateCostDerivativeInputState(dLdux);
  };

  void getTerminalCost(scalar_t &Phi) override {
    systemCost_->getTerminalCost(Phi);
  };

  virtual void getTerminalCostDerivativeTime(scalar_t &dPhidt) { dPhidt = 0; }

  void getTerminalCostDerivativeState(state_vector_t &dPhidx) override {
    system_state_vector_t dPhidx_system;
    systemCost_->getTerminalCostDerivativeState(dPhidx_system);
    dPhidx.segment(0, SYSTEM_STATE_DIM) = dPhidx_system;
    dPhidx.segment(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
  };

  void getTerminalCostSecondDerivativeState(state_matrix_t &dPhidxx) override {
    system_state_matrix_t dPhidxx_system;
    systemCost_->getTerminalCostSecondDerivativeState(dPhidxx_system);
    dPhidxx.setZero();
    dPhidxx.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = dPhidxx_system;
  };

 protected:
  cost_desired_trajectories_t systemCostDesiredTrajectories_;
  using BASE::costDesiredTrajectoriesPtr_;
  using BASE::xNominalFunc_;
  using BASE::uNominalFunc_;

 private:
  std::shared_ptr<SYSTEMCOST> systemCost_;
  std::unique_ptr<LoopshapingCostImplementation> loopshapingCostImplementation_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};
}; // namespace ocs2

#endif //OCS2_LOOPSHAPINGCOST_H
