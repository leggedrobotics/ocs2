/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/cost/StateCostCollection.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateCostCollection::StateCostCollection(const StateCostCollection& other) : StateCost(other) {
  // Loop through all costs by name and clone into the new object
  costTermMap_.clear();
  for (const auto& costPair : other.costTermMap_) {
    add(costPair.first, std::unique_ptr<StateCost>(costPair.second->clone()));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateCostCollection::StateCostCollection(StateCostCollection&& other) noexcept
    : StateCost(other), costTermMap_(std::move(other.costTermMap_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateCostCollection& StateCostCollection::operator=(const StateCostCollection& rhs) {
  *this = StateCostCollection(rhs);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateCostCollection& StateCostCollection::operator=(StateCostCollection&& rhs) {
  std::swap(costTermMap_, rhs.costTermMap_);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateCostCollection* StateCostCollection::clone() const {
  return new StateCostCollection(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void StateCostCollection::add(std::string name, std::unique_ptr<StateCost> costTerm) {
  auto info = costTermMap_.emplace(std::move(name), std::move(costTerm));
  if (!info.second) {
    throw std::runtime_error(std::string("[StateCostCollection::add] Cost term with name \"") + info.first->first + "\" already exists");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateCostCollection::getValue(scalar_t time, const vector_t& state, const CostDesiredTrajectories& desiredTrajectory,
                                       const PreComputation* preCompPtr) const {
  scalar_t cost = 0.0;

  // accumulate cost terms
  for (const auto& costPair : costTermMap_) {
    if (costPair.second->isActive()) {
      cost += costPair.second->getValue(time, state, desiredTrajectory, preCompPtr);
    }
  }

  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateCostCollection::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                    const CostDesiredTrajectories& desiredTrajectory,
                                                                                    const PreComputation* preCompPtr) const {
  auto cost = ScalarFunctionQuadraticApproximation::Zero(state.rows(), 0);

  // accumulate cost term quadratic approximation
  for (const auto& costPair : costTermMap_) {
    if (costPair.second->isActive()) {
      const auto costTermApproximation = costPair.second->getQuadraticApproximation(time, state, desiredTrajectory, preCompPtr);
      cost.f += costTermApproximation.f;
      cost.dfdx += costTermApproximation.dfdx;
      cost.dfdxx += costTermApproximation.dfdxx;
    }
  }

  return cost;
}

}  // namespace ocs2
