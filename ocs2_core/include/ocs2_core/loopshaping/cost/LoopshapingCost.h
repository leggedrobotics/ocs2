//
// Created by ruben on 24.09.18.
//

#ifndef OCS2_LOOPSHAPINGCOST_H
#define OCS2_LOOPSHAPINGCOST_H

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/cost/QuadraticCostFunction.h"
#include "ocs2_core/loopshaping/LoopshapingDefinition.h"

namespace ocs2 {

template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
class LoopshapingCost : public CostFunctionBase<FULL_STATE_DIM, FULL_INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LoopshapingCost>;
  using ConstPtr = std::shared_ptr<const LoopshapingCost>;

  using BASE = CostFunctionBase<FULL_STATE_DIM, FULL_INPUT_DIM>;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  using SYSTEMCOST = CostFunctionBase<SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM>;
  using system_state_vector_t = typename SYSTEMCOST::state_vector_t;
  using system_input_vector_t = typename SYSTEMCOST::input_vector_t;
  using system_state_matrix_t = typename SYSTEMCOST::state_matrix_t;
  using system_input_matrix_t = typename SYSTEMCOST::input_matrix_t;
  using system_input_state_matrix_t = typename SYSTEMCOST::input_state_matrix_t;

  using filter_state_vector_t = Eigen::Matrix<scalar_t, FILTER_STATE_DIM, 1>;
  using filter_input_vector_t = Eigen::Matrix<scalar_t, FILTER_INPUT_DIM, 1>;

  ~LoopshapingCost() override = default;

  LoopshapingCost(const LoopshapingCost& obj)
      : BASE(),
        systemCost_(obj.systemCost_->clone()),
        loopshapingDefinition_(obj.loopshapingDefinition_),
        costApproximationValid_(false),
        costEvaluationValid_(false) {}

  static std::unique_ptr<LoopshapingCost> create(const SYSTEMCOST& systemCost,
                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
    costApproximationValid_ = false;
    costEvaluationValid_ = false;

    t_ = t;
    loopshapingDefinition_->getSystemState(x, x_system_);
    loopshapingDefinition_->getSystemInput(x, u, u_system_);
    loopshapingDefinition_->getFilterState(x, x_filter_);
    loopshapingDefinition_->getFilteredInput(x, u, u_filter_);

    BASE::setCurrentStateAndControl(t, x, u);
  }

  void setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) override {
    if (costDesiredTrajectoriesPtr) {
      BASE::setCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);

      // Desired trajectories are dynamic size -> must resize for future cast to fixed size vectors
      size_t reference_length = costDesiredTrajectoriesPtr->desiredTimeTrajectory().size();
      systemCostDesiredTrajectories_ = CostDesiredTrajectories(reference_length);
      systemCostDesiredTrajectories_.desiredTimeTrajectory() = costDesiredTrajectoriesPtr->desiredTimeTrajectory();
      auto& systemStateTrajectory = systemCostDesiredTrajectories_.desiredStateTrajectory();
      auto& systemInputTrajectory = systemCostDesiredTrajectories_.desiredInputTrajectory();
      const auto& stateTrajectory = costDesiredTrajectoriesPtr->desiredStateTrajectory();
      const auto& inputTrajectory = costDesiredTrajectoriesPtr->desiredInputTrajectory();
      for (int k = 0; k < reference_length; k++) {
        // For now assume that cost DesiredTrajectory is specified w.r.t original system x, u
        systemStateTrajectory[k] = stateTrajectory[k].segment(0, SYSTEM_STATE_DIM);
        systemInputTrajectory[k] = inputTrajectory[k].segment(0, SYSTEM_INPUT_DIM);
      }

      systemCost_->setCostDesiredTrajectoriesPtr(&systemCostDesiredTrajectories_);
    }
  }

  void getIntermediateCost(scalar_t& L) override {
    computeCost();
    const auto& gamma = loopshapingDefinition_->gamma_;
    L = gamma * c_filter_ + (1.0 - gamma) * c_system_;
  };

  void getIntermediateCostDerivativeTime(scalar_t& dLdt) override {
    computeApproximation();
    // TODO
    dLdt = 0;
  }

  void getTerminalCost(scalar_t& Phi) override {
    systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
    systemCost_->getTerminalCost(Phi);
  };

  void getTerminalCostDerivativeTime(scalar_t& dPhidt) override { dPhidt = 0; }

  void getTerminalCostDerivativeState(state_vector_t& dPhidx) override {
    system_state_vector_t dPhidx_system;
    systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
    systemCost_->getTerminalCostDerivativeState(dPhidx_system);
    dPhidx.segment(0, SYSTEM_STATE_DIM) = dPhidx_system;
    dPhidx.segment(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
  };

  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override {
    system_state_matrix_t dPhidxx_system;
    systemCost_->setCurrentStateAndControl(t_, x_system_, u_system_);
    systemCost_->getTerminalCostSecondDerivativeState(dPhidxx_system);
    dPhidxx.setZero();
    dPhidxx.block(0, 0, SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = dPhidxx_system;
  };

 protected:
  LoopshapingCost(const SYSTEMCOST& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(), systemCost_(systemCost.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  CostDesiredTrajectories systemCostDesiredTrajectories_;
  using BASE::costDesiredTrajectoriesPtr_;

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
  std::unique_ptr<SYSTEMCOST> systemCost_;

  bool costApproximationValid_;
  bool costEvaluationValid_;
};
};  // namespace ocs2

// include derived implementations to be dispatched by the factory method
#include "LoopshapingCostEliminatePattern.h"
#include "LoopshapingCostInputPattern.h"
#include "LoopshapingCostOutputPattern.h"

// Implement factory method
namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
std::unique_ptr<LoopshapingCost<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>>
LoopshapingCost<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>::create(
    const SYSTEMCOST& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingCost>(
          new LoopshapingCostOutputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                           FILTER_INPUT_DIM>(systemCost, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingCost>(
          new LoopshapingCostInputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                          FILTER_INPUT_DIM>(systemCost, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingCost>(
          new LoopshapingCostEliminatePattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM,
                                              FILTER_INPUT_DIM>(systemCost, std::move(loopshapingDefinition)));
  }
}
}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGCOST_H
