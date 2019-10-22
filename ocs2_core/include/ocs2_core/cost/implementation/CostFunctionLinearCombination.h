/*
 * Implementation CostFunctionLinearCombination.h
 *
 *  Created on: September, 2019
 *      Author: Johannes Pankert
 */

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::CostFunctionLinearCombination(const std::vector<WeightedCost>& weightedCosts)
    : weightedCosts_(weightedCosts) {}

template <size_t STATE_DIM, size_t INPUT_DIM>
CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::CostFunctionLinearCombination(const CostFunctionLinearCombination& rhs)
    : BASE(rhs), weightedCosts_() {
  weightedCosts_.resize(rhs.weightedCosts_.size());
  for (int i = 0; i < rhs.weightedCosts_.size(); i++) {
    weightedCosts_[i].first = rhs.weightedCosts_[i].first;
    weightedCosts_[i].second.reset(rhs.weightedCosts_[i].second->clone());
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM>
CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>* CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::clone() const {
  return new CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>(*this);
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getIntermediateCost(scalar_t& L) {
  L = 0;
  for (int i = 0; i < weightedCosts_.size(); i++) {
    scalar_t LSingle = 0;
    weightedCosts_[i].second->getIntermediateCost(LSingle);
    L += weightedCosts_[i].first * LSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeState(state_vector_t& dLdx) {
  dLdx = state_vector_t::Zero();
  for (int i = 0; i < weightedCosts_.size(); i++) {
    state_vector_t dLdxSingle = state_vector_t::Zero();
    weightedCosts_[i].second->getIntermediateCostDerivativeState(dLdxSingle);
    dLdx += weightedCosts_[i].first * dLdxSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) {
  dLdxx = state_matrix_t::Zero();
  for (int i = 0; i < weightedCosts_.size(); i++) {
    state_matrix_t dLdxxSingle = state_matrix_t::Zero();
    weightedCosts_[i].second->getIntermediateCostSecondDerivativeState(dLdxxSingle);
    dLdxx += weightedCosts_[i].first * dLdxxSingle;
  }
}

template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeInput(input_vector_t& dLdu) {
  dLdu = input_vector_t::Zero();
  for (int i = 0; i < weightedCosts_.size(); i++) {
    input_vector_t dLduSingle = input_vector_t::Zero();
    weightedCosts_[i].second->getIntermediateCostDerivativeInput(dLduSingle);
    dLdu += weightedCosts_[i].first * dLduSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) {
  dLduu = input_matrix_t::Zero();
  for (int i = 0; i < weightedCosts_.size(); i++) {
    input_matrix_t dLduuSingle = input_matrix_t::Zero();
    weightedCosts_[i].second->getIntermediateCostSecondDerivativeInput(dLduuSingle);
    dLduu += weightedCosts_[i].first * dLduuSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdux) {
  dLdux = input_state_matrix_t::Zero();
  for (int i = 0; i < weightedCosts_.size(); i++) {
    input_state_matrix_t dLduxSingle = input_state_matrix_t::Zero();
    weightedCosts_[i].second->getIntermediateCostDerivativeInputState(dLduxSingle);
    dLdux += weightedCosts_[i].first * dLduxSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getTerminalCost(scalar_t& Phi) {
  Phi = 0;
  for (int i = 0; i < weightedCosts_.size(); i++) {
    scalar_t PhiSingle = 0;
    weightedCosts_[i].second->getTerminalCost(PhiSingle);
    Phi += weightedCosts_[i].first * PhiSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getTerminalCostDerivativeState(state_vector_t& dPhidx) {
  dPhidx = state_vector_t::Zero();
  for (int i = 0; i < weightedCosts_.size(); i++) {
    state_vector_t dPhidxSingle = state_vector_t::Zero();
    weightedCosts_[i].second->getTerminalCostDerivativeState(dPhidxSingle);
    dPhidx += weightedCosts_[i].first * dPhidxSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) {
  dPhidxx = state_matrix_t::Zero();
  for (int i = 0; i < weightedCosts_.size(); i++) {
    state_matrix_t dPhidxxSingle = state_matrix_t::Zero();
    weightedCosts_[i].second->getTerminalCostSecondDerivativeState(dPhidxxSingle);
    dPhidxx += weightedCosts_[i].first * dPhidxxSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::setCostDesiredTrajectories(
    const cost_desired_trajectories_t& costDesiredTrajectories) {
  BASE::setCostDesiredTrajectories(costDesiredTrajectories);
  for (int i = 0; i < weightedCosts_.size(); i++) {
    weightedCosts_[i].second->setCostDesiredTrajectories(costDesiredTrajectories);
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x,
                                                                                    const input_vector_t& u) {
  BASE::setCurrentStateAndControl(t, x, u);
  for (int i = 0; i < weightedCosts_.size(); i++) {
    weightedCosts_[i].second->setCurrentStateAndControl(t, x, u);
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getIntermediateCostDerivativeTime(scalar_t& dLdt) {
  dLdt = 0;
  for (int i = 0; i < weightedCosts_.size(); i++) {
    scalar_t dLdtSingle = 0;
    weightedCosts_[i].second->getIntermediateCostDerivativeTime(dLdtSingle);
    dLdt += weightedCosts_[i].first * dLdtSingle;
  }
}
template <size_t STATE_DIM, size_t INPUT_DIM>
void CostFunctionLinearCombination<STATE_DIM, INPUT_DIM>::getTerminalCostDerivativeTime(scalar_t& dPhidt) {
  dPhidt = 0;
  for (int i = 0; i < weightedCosts_.size(); i++) {
    scalar_t dPhidtSingle = 0;
    weightedCosts_[i].second->getTerminalCostDerivativeTime(dPhidtSingle);
    dPhidt += weightedCosts_[i].first * dPhidtSingle;
  }
}

}  // namespace ocs2