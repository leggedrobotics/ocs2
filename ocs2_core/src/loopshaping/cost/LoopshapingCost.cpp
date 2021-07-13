/******************************************************************************
Copyright (c) 2020, Ruben Grandia. All rights reserved.

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

#include <ocs2_core/loopshaping/cost/LoopshapingCost.h>
#include <ocs2_core/loopshaping/cost/LoopshapingCostEliminatePattern.h>
#include <ocs2_core/loopshaping/cost/LoopshapingCostInputPattern.h>
#include <ocs2_core/loopshaping/cost/LoopshapingCostOutputPattern.h>

namespace ocs2 {

std::unique_ptr<LoopshapingCost> LoopshapingCost::create(const CostFunctionBase& systemCost,
                                                         std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingCost>(new LoopshapingCostOutputPattern(systemCost, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingCost>(new LoopshapingCostInputPattern(systemCost, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingCost>(new LoopshapingCostEliminatePattern(systemCost, std::move(loopshapingDefinition)));
    default:
      throw std::runtime_error("[LoopshapingCost::create] invalid loopshaping type");
  }
}

void LoopshapingCost::setTargetTrajectoriesPtr(const TargetTrajectories* targetTrajectoriesPtr) {
  if (targetTrajectoriesPtr != nullptr) {
    CostFunctionBase::setTargetTrajectoriesPtr(targetTrajectoriesPtr);

    // For now assume that cost DesiredTrajectory is specified w.r.t original system x, u
    systemCost_->setTargetTrajectoriesPtr(targetTrajectoriesPtr);
  }
}

scalar_t LoopshapingCost::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const vector_t u_system = loopshapingDefinition_->getSystemInput(x, u);
  const vector_t u_filter = loopshapingDefinition_->getFilteredInput(x, u);

  const scalar_t L_system = systemCost_->cost(t, x_system, u_system);
  const scalar_t L_filter = systemCost_->cost(t, x_system, u_filter);
  const scalar_t gamma = loopshapingDefinition_->gamma_;

  return gamma * L_filter + (1.0 - gamma) * L_system;
}

scalar_t LoopshapingCost::costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) {
  // TODO(rgrandia)
  return 0;
}

scalar_t LoopshapingCost::finalCost(scalar_t t, const vector_t& x) {
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  return systemCost_->finalCost(t, x_system);
};

scalar_t LoopshapingCost::finalCostDerivativeTime(scalar_t t, const vector_t& x) {
  return 0;
}

ScalarFunctionQuadraticApproximation LoopshapingCost::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  const vector_t x_system = loopshapingDefinition_->getSystemState(x);
  const auto Phi_system = systemCost_->finalCostQuadraticApproximation(t, x_system);
  const size_t FILTER_STATE_DIM = loopshapingDefinition_->getInputFilter().getNumStates();

  ScalarFunctionQuadraticApproximation Phi;
  Phi.f = Phi_system.f;
  Phi.dfdx.resize(x.rows());
  Phi.dfdx.head(x_system.rows()) = Phi_system.dfdx;
  Phi.dfdx.tail(FILTER_STATE_DIM).setZero();
  Phi.dfdxx.setZero(x.rows(), x.rows());
  Phi.dfdxx.topLeftCorner(x_system.rows(), x_system.rows()) = Phi_system.dfdxx;
  return Phi;
}

}  // namespace ocs2
