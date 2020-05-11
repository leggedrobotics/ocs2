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
void CostFunctionLinearCombination::getIntermediateCost(scalar_t& L) {
  L = 0;
  for (auto& weightedCost : weightedCosts_) {
    scalar_t LSingle = 0;
    weightedCost.second->getIntermediateCost(LSingle);
    L += weightedCost.first * LSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getIntermediateCostDerivativeState(vector_t& dLdx) {
  dLdx = vector_t::Zero(x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    vector_t dLdxSingle = vector_t::Zero(x_.rows());
    weightedCost.second->getIntermediateCostDerivativeState(dLdxSingle);
    dLdx += weightedCost.first * dLdxSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) {
  dLdxx = matrix_t::Zero(x_.rows(), x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    matrix_t dLdxxSingle = matrix_t::Zero(x_.rows(), x_.rows());
    weightedCost.second->getIntermediateCostSecondDerivativeState(dLdxxSingle);
    dLdxx += weightedCost.first * dLdxxSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getIntermediateCostDerivativeInput(vector_t& dLdu) {
  dLdu = vector_t::Zero(u_.rows());
  for (auto& weightedCost : weightedCosts_) {
    vector_t dLduSingle = vector_t::Zero(u_.rows());
    weightedCost.second->getIntermediateCostDerivativeInput(dLduSingle);
    dLdu += weightedCost.first * dLduSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getIntermediateCostSecondDerivativeInput(matrix_t& dLduu) {
  dLduu = matrix_t::Zero(u_.rows(), u_.rows());
  for (auto& weightedCost : weightedCosts_) {
    matrix_t dLduuSingle = matrix_t::Zero(u_.rows(), u_.rows());
    weightedCost.second->getIntermediateCostSecondDerivativeInput(dLduuSingle);
    dLduu += weightedCost.first * dLduuSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getIntermediateCostDerivativeInputState(matrix_t& dLdux) {
  dLdux = matrix_t::Zero(u_.rows(), x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    matrix_t dLduxSingle = matrix_t::Zero(u_.rows(), x_.rows());
    weightedCost.second->getIntermediateCostDerivativeInputState(dLduxSingle);
    dLdux += weightedCost.first * dLduxSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getTerminalCost(scalar_t& Phi) {
  Phi = 0;
  for (auto& weightedCost : weightedCosts_) {
    scalar_t PhiSingle = 0;
    weightedCost.second->getTerminalCost(PhiSingle);
    Phi += weightedCost.first * PhiSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getTerminalCostDerivativeState(vector_t& dPhidx) {
  dPhidx = vector_t::Zero(x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    vector_t dPhidxSingle = vector_t::Zero(x_.rows());
    weightedCost.second->getTerminalCostDerivativeState(dPhidxSingle);
    dPhidx += weightedCost.first * dPhidxSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) {
  dPhidxx = matrix_t::Zero(x_.rows(), x_.rows());
  for (auto& weightedCost : weightedCosts_) {
    matrix_t dPhidxxSingle = matrix_t::Zero(x_.rows(), x_.rows());
    weightedCost.second->getTerminalCostSecondDerivativeState(dPhidxxSingle);
    dPhidxx += weightedCost.first * dPhidxxSingle;
  }
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
void CostFunctionLinearCombination::setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) {
  CostFunctionBase::setCurrentStateAndControl(t, x, u);
  for (auto& weightedCost : weightedCosts_) {
    weightedCost.second->setCurrentStateAndControl(t, x, u);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getIntermediateCostDerivativeTime(scalar_t& dLdt) {
  dLdt = 0;
  for (auto& weightedCost : weightedCosts_) {
    scalar_t dLdtSingle = 0;
    weightedCost.second->getIntermediateCostDerivativeTime(dLdtSingle);
    dLdt += weightedCost.first * dLdtSingle;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostFunctionLinearCombination::getTerminalCostDerivativeTime(scalar_t& dPhidt) {
  dPhidt = 0;
  for (auto& weightedCost : weightedCosts_) {
    scalar_t dPhidtSingle = 0;
    weightedCost.second->getTerminalCostDerivativeTime(dPhidtSingle);
    dPhidt += weightedCost.first * dPhidtSingle;
  }
}

}  // namespace ocs2
