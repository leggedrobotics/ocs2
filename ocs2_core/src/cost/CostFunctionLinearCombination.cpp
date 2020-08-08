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
CostFunctionLinearCombination::CostFunctionLinearCombination(std::vector<WeightedCost> weightedCosts)
    : weightedCosts_(std::move(weightedCosts)) {}

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
void CostFunctionLinearCombination::setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) {
  CostFunctionBase::setCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);
  for (auto& weightedCost : weightedCosts_) {
    weightedCost.second->setCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionLinearCombination::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  scalar_t L = 0.0;
  for (auto& weightedCost : weightedCosts_) {
    L += weightedCost.first * weightedCost.second->cost(t, x, u);
  }
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionLinearCombination::finalCost(scalar_t t, const vector_t& x) {
  scalar_t Phi = 0.0;
  for (auto& weightedCost : weightedCosts_) {
    Phi += weightedCost.first * weightedCost.second->finalCost(t, x);
  }
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostFunctionLinearCombination::costQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                               const vector_t& u) {
  ScalarFunctionQuadraticApproximation L;
  L.f = 0.0;
  L.dfdx.setZero(x.rows());
  L.dfdu.setZero(u.rows());
  L.dfdxx.setZero(x.rows(), x.rows());
  L.dfdux.setZero(u.rows(), x.rows());
  L.dfduu.setZero(u.rows(), u.rows());
  for (auto& weightedCost : weightedCosts_) {
    const auto cost = weightedCost.second->costQuadraticApproximation(t, x, u);
    L.f += weightedCost.first * cost.f;
    L.dfdx += weightedCost.first * cost.dfdx;
    L.dfdu += weightedCost.first * cost.dfdu;
    L.dfdxx += weightedCost.first * cost.dfdxx;
    L.dfdux += weightedCost.first * cost.dfdux;
    L.dfduu += weightedCost.first * cost.dfduu;
  }
  return L;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostFunctionLinearCombination::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = 0.0;
  Phi.dfdx.setZero(x.rows());
  Phi.dfdxx.setZero(x.rows(), x.rows());
  for (auto& weightedCost : weightedCosts_) {
    const auto cost = weightedCost.second->finalCostQuadraticApproximation(t, x);
    Phi.f += weightedCost.first * cost.f;
    Phi.dfdx += weightedCost.first * cost.dfdx;
    Phi.dfdxx += weightedCost.first * cost.dfdxx;
  }
  return Phi;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionLinearCombination::costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  scalar_t dLdt = 0;
  for (auto& weightedCost : weightedCosts_) {
    dLdt += weightedCost.first * weightedCost.second->costDerivativeTime(t, x, u);
  }
  return dLdt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionLinearCombination::finalCostDerivativeTime(scalar_t t, const vector_t& x) {
  scalar_t dPhidt = 0;
  for (auto& weightedCost : weightedCosts_) {
    dPhidt += weightedCost.first * weightedCost.second->finalCostDerivativeTime(t, x);
  }
  return dPhidt;
}

}  // namespace ocs2
