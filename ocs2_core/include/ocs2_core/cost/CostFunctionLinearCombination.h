/*
 * CostFunctionLinearCombination.h
 *
 *  Created on: September, 2019
 *      Author: Johannes Pankert
 */

#pragma once

#include <ocs2_core/cost/CostFunctionBase.h>
#include <utility>
#include "ocs2_core/Dimensions.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class CostFunctionLinearCombination : public ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using typename BASE::scalar_t;
  using WeightedCost = std::pair<scalar_t, std::shared_ptr<BASE>>;
  using typename BASE::cost_desired_trajectories_t;
  using typename BASE::dynamic_vector_array_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  CostFunctionLinearCombination(const std::vector<WeightedCost>& weightedCosts);

  CostFunctionLinearCombination(const CostFunctionLinearCombination& rhs);

  CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>* clone() const override;
  void getIntermediateCost(scalar_t& L) override;
  void getIntermediateCostDerivativeState(state_vector_t& dLdx) override;
  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) override;
  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) override;
  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) override;
  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) override;
  void getTerminalCost(scalar_t& Phi) override;
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) override;
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) override;
  void setCostDesiredTrajectories(const cost_desired_trajectories_t& costDesiredTrajectories) override;
  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;
  void getIntermediateCostDerivativeTime(scalar_t& dLdt) override;
  void getTerminalCostDerivativeTime(scalar_t& dPhidt) override;

 protected:
  CostFunctionLinearCombination() = default;

 protected:
  std::vector<WeightedCost> weightedCosts_;
};

}  // namespace ocs2

#include "implementation/CostFunctionLinearCombination.h"