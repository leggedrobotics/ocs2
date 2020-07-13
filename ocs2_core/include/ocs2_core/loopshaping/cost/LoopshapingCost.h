//
// Created by ruben on 24.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

class LoopshapingCost : public CostFunctionBase {
 public:
  ~LoopshapingCost() override = default;

  LoopshapingCost(const LoopshapingCost& obj)
      : CostFunctionBase(),
        systemCost_(obj.systemCost_->clone()),
        loopshapingDefinition_(obj.loopshapingDefinition_),
        costApproximationValid_(false),
        costEvaluationValid_(false) {}

  static std::unique_ptr<LoopshapingCost> create(const CostFunctionBase& systemCost,
                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) override;

  void setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) override;

  scalar_t getCost() override;

  scalar_t getCostDerivativeTime() override;

  scalar_t getTerminalCost() override;

  scalar_t getTerminalCostDerivativeTime() override;

  vector_t getTerminalCostDerivativeState() override;

  matrix_t getTerminalCostSecondDerivativeState() override;

 protected:
  LoopshapingCost(const CostFunctionBase& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : CostFunctionBase(), systemCost_(systemCost.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  CostDesiredTrajectories systemCostDesiredTrajectories_;
  using CostFunctionBase::costDesiredTrajectoriesPtr_;

  scalar_t t_;
  vector_t x_filter_;
  vector_t u_filter_;
  vector_t x_system_;
  vector_t u_system_;

  matrix_t Q_system_;
  matrix_t R_system_;
  matrix_t P_system_;
  vector_t q_system_;
  vector_t r_system_;
  scalar_t c_system_;

  matrix_t Q_filter_;
  matrix_t R_filter_;
  matrix_t P_filter_;
  vector_t q_filter_;
  vector_t r_filter_;
  scalar_t c_filter_;

  void computeCost();

  void computeApproximation();

 private:
  std::unique_ptr<CostFunctionBase> systemCost_;

  bool costApproximationValid_;
  bool costEvaluationValid_;
};
};  // namespace ocs2
