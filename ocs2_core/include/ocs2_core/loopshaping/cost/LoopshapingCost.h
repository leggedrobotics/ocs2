//
// Created by ruben on 24.09.18.
//

#ifndef OCS2_LOOPSHAPINGCOST_H
#define OCS2_LOOPSHAPINGCOST_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/cost/QuadraticCostFunction.h"
#include "ocs2_core/logic/rules/NullLogicRules.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {

template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T=NullLogicRules>
class LoopshapingCost : public CostFunctionBase<FULL_STATE_DIM, FULL_INPUT_DIM, LOGIC_RULES_T> {
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

  ~LoopshapingCost() override = default;

  void initializeModel(
      LogicRulesMachine<LOGIC_RULES_T> &logicRulesMachine,
      const size_t &partitionIndex,
      const char *algorithmName = NULL) override {
    BASE::initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    systemCost_->initializeModel(logicRulesMachine, partitionIndex, algorithmName);
    costApproximationValid_ = false;
    costEvaluationValid_ = false;
  }

  static std::unique_ptr<LoopshapingCost> Create(const SYSTEMCOST &systemCost,
                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void setCurrentStateAndControl(
      const scalar_t &t,
      const state_vector_t &x,
      const input_vector_t &u) override {
    costApproximationValid_ = false;
    costEvaluationValid_ = false;

    dynamic_vector_t xNominal;
    xNominalFunc_.interpolate(t, xNominal);
    dynamic_vector_t uNominal;
    uNominalFunc_.interpolate(t, uNominal);

    loopshapingDefinition_->getSystemState(x, x_system_);
    loopshapingDefinition_->getSystemInput(x, u, u_system_);
    loopshapingDefinition_->getFilterState(x, x_filter_);
    loopshapingDefinition_->getFilteredInput(x, u, u_filter_);

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
    computeCost();
    const auto &gamma = loopshapingDefinition_->gamma;
    L = gamma * c_filter_ + (1.0 - gamma) * c_system_;
  };

  void getIntermediateCostDerivativeTime(scalar_t &dLdt) override {
    computeApproximation();
    // TODO
    dLdt = 0;
  }

  void getTerminalCost(scalar_t &Phi) override {
    systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
    systemCost_->getTerminalCost(Phi);
  };

  void getTerminalCostDerivativeTime(scalar_t &dPhidt) override { dPhidt = 0; }

  void getTerminalCostDerivativeState(state_vector_t &dPhidx) override {
    system_state_vector_t dPhidx_system;
    systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
    systemCost_->getTerminalCostDerivativeState(dPhidx_system);
    dPhidx.segment(0, SYSTEM_STATE_DIM) = dPhidx_system;
    dPhidx.segment(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
  };

  void getTerminalCostSecondDerivativeState(state_matrix_t &dPhidxx) override {
    system_state_matrix_t dPhidxx_system;
    systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
    systemCost_->getTerminalCostSecondDerivativeState(dPhidxx_system);
    dPhidxx.setZero();
    dPhidxx.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = dPhidxx_system;
  };

 protected:
  LoopshapingCost(const SYSTEMCOST &systemCost,
                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(),
        systemCost_(systemCost.clone()),
        loopshapingDefinition_(std::move(loopshapingDefinition)) {
  }

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  cost_desired_trajectories_t systemCostDesiredTrajectories_;
  using BASE::costDesiredTrajectoriesPtr_;
  using BASE::xNominalFunc_;
  using BASE::uNominalFunc_;

  scalar_t t_;
  filter_state_vector_t x_filter_;
  filter_input_vector_t u_filter_;
  system_state_vector_t x_system_;
  system_input_vector_t u_system_;

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

  void computeCost() {
    if (!costEvaluationValid_) {
      systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
      systemCost_->getIntermediateCost(c_system_);

      systemCost_->setCurrentStateAndControl(t_, x_system_, u_filter_);
      systemCost_->getIntermediateCost(c_filter_);

      costEvaluationValid_ = true;
    }
  };

  void computeApproximation() {
    if (!costApproximationValid_) {
      systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
      systemCost_->getIntermediateCostSecondDerivativeState(Q_system_);
      systemCost_->getIntermediateCostDerivativeInputState(P_system_);
      systemCost_->getIntermediateCostSecondDerivativeInput(R_system_);
      systemCost_->getIntermediateCostDerivativeState(q_system_);
      systemCost_->getIntermediateCostDerivativeInput(r_system_);
      systemCost_->getIntermediateCost(c_system_);

      systemCost_->setCurrentStateAndControl(t_, x_system_, u_filter_);
      systemCost_->getIntermediateCostSecondDerivativeState(Q_filter_);
      systemCost_->getIntermediateCostDerivativeInputState(P_filter_);
      systemCost_->getIntermediateCostSecondDerivativeInput(R_filter_);
      systemCost_->getIntermediateCostDerivativeState(q_filter_);
      systemCost_->getIntermediateCostDerivativeInput(r_filter_);
      systemCost_->getIntermediateCost(c_filter_);

      costEvaluationValid_ = true;
      costApproximationValid_ = true;
    }
  };

 private:
  std::shared_ptr<SYSTEMCOST> systemCost_;

  bool costApproximationValid_;
  bool costEvaluationValid_;
};
}; // namespace ocs2

// include derived implementations to be dispatched by the factory method
#include "LoopshapingCostInputPattern.h"
#include "LoopshapingCostOutputPattern.h"

// Implement factory method
namespace ocs2 {
template<size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM,
    size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM,
    size_t FILTER_STATE_DIM, size_t FILTER_INPUT_DIM,
    class LOGIC_RULES_T>
std::unique_ptr<LoopshapingCost<FULL_STATE_DIM,
                                FULL_INPUT_DIM,
                                SYSTEM_STATE_DIM,
                                SYSTEM_INPUT_DIM,
                                FILTER_STATE_DIM,
                                FILTER_INPUT_DIM,
                                LOGIC_RULES_T>>
LoopshapingCost<FULL_STATE_DIM,
                FULL_INPUT_DIM,
                SYSTEM_STATE_DIM,
                SYSTEM_INPUT_DIM,
                FILTER_STATE_DIM,
                FILTER_INPUT_DIM,
                LOGIC_RULES_T>::Create(const SYSTEMCOST &systemCost,
                                       std::shared_ptr<
                                           LoopshapingDefinition> loopshapingDefinition) {
  if (loopshapingDefinition->getInputFilter_s().getNumOutputs() > 0) {
    return std::unique_ptr<LoopshapingCost>(new LoopshapingCostInputPattern<FULL_STATE_DIM,
                                                                            FULL_INPUT_DIM,
                                                                            SYSTEM_STATE_DIM,
                                                                            SYSTEM_INPUT_DIM,
                                                                            FILTER_STATE_DIM,
                                                                            FILTER_INPUT_DIM,
                                                                            LOGIC_RULES_T>(systemCost,
                                                                                            std::move(
                                                                                                loopshapingDefinition)));
  } else if (loopshapingDefinition->getInputFilter_r().getNumOutputs() > 0) {
    return std::unique_ptr<LoopshapingCost>(
        new LoopshapingCostOutputPattern<FULL_STATE_DIM,
                                         FULL_INPUT_DIM,
                                         SYSTEM_STATE_DIM,
                                         SYSTEM_INPUT_DIM,
                                         FILTER_STATE_DIM,
                                         FILTER_INPUT_DIM,
                                         LOGIC_RULES_T>(systemCost,
                                                         std::move(loopshapingDefinition)));
  }
}
} // namespace ocs2



#endif //OCS2_LOOPSHAPINGCOST_H
