/******************************************************************************
Copyright (c) 2019, Johannes Pankert. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_core/cost/CostFunctionLinearCombination.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionLinearCombination::CostFunctionLinearCombination(const std::vector<WeightedCost>& weightedCosts)
    : weightedCosts_(weightedCosts) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionLinearCombination::CostFunctionLinearCombination(const CostFunctionLinearCombination& rhs) : CostFunctionBase(rhs) {
  weightedCosts_.resize(rhs.weightedCosts_.size());
  for (int i = 0; i < rhs.weightedCosts_.size(); i++) {
    weightedCosts_[i].first = rhs.weightedCosts_[i].first;
    weightedCosts_[i].second.reset(rhs.weightedCosts_[i].second->clone());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionLinearCombination* CostFunctionLinearCombination::clone() const {
  return new CostFunctionLinearCombination(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionLinearCombination::getCost() {
  scalar_t L = 0;
  for (auto& weightedCost : weightedCosts_) {
    L += weightedCost.first * weightedCost.second->getCost();
  }
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CostFunctionLinearCombination::getCostDerivativeState() {
  vector_t dLdx = vector_t::Zero(x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    dLdx += weightedCost.first * weightedCost.second->getCostDerivativeState();
  }
  return dLdx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CostFunctionLinearCombination::getCostSecondDerivativeState() {
  matrix_t dLdxx = matrix_t::Zero(x_.rows(), x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    dLdxx += weightedCost.first * weightedCost.second->getCostSecondDerivativeState();
  }
  return dLdxx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CostFunctionLinearCombination::getCostDerivativeInput() {
  vector_t dLdu = vector_t::Zero(u_.rows());
  for (auto& weightedCost : weightedCosts_) {
    dLdu += weightedCost.first * weightedCost.second->getCostDerivativeInput();
  }
  return dLdu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CostFunctionLinearCombination::getCostSecondDerivativeInput() {
  matrix_t dLduu = matrix_t::Zero(u_.rows(), u_.rows());
  for (auto& weightedCost : weightedCosts_) {
    dLduu += weightedCost.first * weightedCost.second->getCostSecondDerivativeInput();
  }
  return dLduu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CostFunctionLinearCombination::getCostDerivativeInputState() {
  matrix_t dLdux = matrix_t::Zero(u_.rows(), x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    dLdux += weightedCost.first * weightedCost.second->getCostDerivativeInputState();
  }
  return dLdux;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionLinearCombination::getTerminalCost() {
  scalar_t Phi = 0;
  for (auto& weightedCost : weightedCosts_) {
    Phi += weightedCost.first * weightedCost.second->getTerminalCost();
  }
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CostFunctionLinearCombination::getTerminalCostDerivativeState() {
  vector_t dPhidx = vector_t::Zero(x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    dPhidx += weightedCost.first * weightedCost.second->getTerminalCostDerivativeState();
  }
  return dPhidx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CostFunctionLinearCombination::getTerminalCostSecondDerivativeState() {
  matrix_t dPhidxx = matrix_t::Zero(x_.rows(), x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    dPhidxx += weightedCost.first * weightedCost.second->getTerminalCostSecondDerivativeState();
  }
  return dPhidxx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) {
  CostFunctionBase::setCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);
  for (auto& weightedCost : weightedCosts_) {
    weightedCost.second->setCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  CostFunctionBase::setCurrentStateAndControl(t, x, u);
  for (auto& weightedCost : weightedCosts_) {
    weightedCost.second->setCurrentStateAndControl(t, x, u);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionLinearCombination::getCostDerivativeTime() {
  scalar_t dLdt = 0;
  for (auto& weightedCost : weightedCosts_) {
    dLdt += weightedCost.first * weightedCost.second->getCostDerivativeTime();
  }
  return dLdt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionLinearCombination::getTerminalCostDerivativeTime() {
  scalar_t dPhidt = 0;
  for (auto& weightedCost : weightedCosts_) {
    dPhidt += weightedCost.first * weightedCost.second->getTerminalCostDerivativeTime();
  }
  return dPhidt;
}

}  // namespace ocs2
