/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_oc/oc_data/PerformanceIndex.h"

#include <iomanip>

#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

PerformanceIndex& PerformanceIndex::operator+=(const PerformanceIndex& rhs) {
  this->merit += rhs.merit;
  this->cost += rhs.cost;
  this->dualFeasibilitiesSSE += rhs.dualFeasibilitiesSSE;
  this->dynamicsViolationSSE += rhs.dynamicsViolationSSE;
  this->equalityConstraintsSSE += rhs.equalityConstraintsSSE;
  this->inequalityConstraintsSSE += rhs.inequalityConstraintsSSE;
  this->equalityLagrangian += rhs.equalityLagrangian;
  this->inequalityLagrangian += rhs.inequalityLagrangian;
  return *this;
}

PerformanceIndex& PerformanceIndex::operator*=(const scalar_t c) {
  this->merit *= c;
  this->cost *= c;
  this->dualFeasibilitiesSSE *= c;
  this->dynamicsViolationSSE *= c;
  this->equalityConstraintsSSE *= c;
  this->inequalityConstraintsSSE *= c;
  this->equalityLagrangian *= c;
  this->inequalityLagrangian *= c;
  return *this;
}

bool PerformanceIndex::isApprox(const PerformanceIndex& other, const scalar_t prec) const {
  return numerics::almost_eq(this->merit, other.merit, prec) && numerics::almost_eq(this->cost, other.cost, prec) &&
         numerics::almost_eq(this->dualFeasibilitiesSSE, other.dualFeasibilitiesSSE, prec) &&
         numerics::almost_eq(this->dynamicsViolationSSE, other.dynamicsViolationSSE, prec) &&
         numerics::almost_eq(this->equalityConstraintsSSE, other.equalityConstraintsSSE, prec) &&
         numerics::almost_eq(this->inequalityConstraintsSSE, other.inequalityConstraintsSSE, prec) &&
         numerics::almost_eq(this->equalityLagrangian, other.equalityLagrangian, prec) &&
         numerics::almost_eq(this->inequalityLagrangian, other.inequalityLagrangian, prec);
}

void swap(PerformanceIndex& lhs, PerformanceIndex& rhs) {
  std::swap(lhs.merit, rhs.merit);
  std::swap(lhs.cost, rhs.cost);
  std::swap(lhs.dualFeasibilitiesSSE, rhs.dualFeasibilitiesSSE);
  std::swap(lhs.dynamicsViolationSSE, rhs.dynamicsViolationSSE);
  std::swap(lhs.equalityConstraintsSSE, rhs.equalityConstraintsSSE);
  std::swap(lhs.inequalityConstraintsSSE, rhs.inequalityConstraintsSSE);
  std::swap(lhs.equalityLagrangian, rhs.equalityLagrangian);
  std::swap(lhs.inequalityLagrangian, rhs.inequalityLagrangian);
}

PerformanceIndex toPerformanceIndex(const Metrics& m) {
  PerformanceIndex performanceIndex;
  performanceIndex.merit = 0.0;  // left for the solver to fill
  performanceIndex.cost = m.cost;
  performanceIndex.dualFeasibilitiesSSE = 0.0;  // left for the solver to fill
  performanceIndex.dynamicsViolationSSE = getEqConstraintsSSE(m.dynamicsViolation);
  performanceIndex.equalityConstraintsSSE = getEqConstraintsSSE(m.stateEqConstraint) + getEqConstraintsSSE(m.stateInputEqConstraint);
  performanceIndex.inequalityConstraintsSSE =
      getIneqConstraintsSSE(m.stateIneqConstraint) + getIneqConstraintsSSE(m.stateInputIneqConstraint);
  performanceIndex.equalityLagrangian = sumPenalties(m.stateEqLagrangian) + sumPenalties(m.stateInputEqLagrangian);
  performanceIndex.inequalityLagrangian = sumPenalties(m.stateIneqLagrangian) + sumPenalties(m.stateInputIneqLagrangian);
  return performanceIndex;
}

PerformanceIndex toPerformanceIndex(const Metrics& m, const scalar_t dt) {
  auto performanceIndex = toPerformanceIndex(m);
  //  performanceIndex.cost *= dt  no need since it is already considered in multiple_shooting::computeIntermediateMetrics()
  performanceIndex.dualFeasibilitiesSSE *= dt;
  performanceIndex.dynamicsViolationSSE *= dt;
  performanceIndex.equalityConstraintsSSE *= dt;
  performanceIndex.inequalityConstraintsSSE *= dt;
  return performanceIndex;
}

std::ostream& operator<<(std::ostream& stream, const PerformanceIndex& performanceIndex) {
  const size_t tabSpace = 12;
  const auto indentation = stream.width();
  stream << std::left;  // fill from left

  stream << std::setw(indentation) << "";
  stream << "Rollout Merit:              " << std::setw(tabSpace) << performanceIndex.merit;
  stream << "Rollout Cost:               " << std::setw(tabSpace) << performanceIndex.cost << '\n';

  stream << std::setw(indentation) << "";
  stream << "Dual feasibilities SSE:     " << std::setw(tabSpace) << performanceIndex.dualFeasibilitiesSSE;
  stream << "Dynamics violation SSE:     " << std::setw(tabSpace) << performanceIndex.dynamicsViolationSSE << '\n';

  stream << std::setw(indentation) << "";
  stream << "Equality constraints SSE:   " << std::setw(tabSpace) << performanceIndex.equalityConstraintsSSE;
  stream << "Inequality constraints SSE: " << std::setw(tabSpace) << performanceIndex.inequalityConstraintsSSE << '\n';

  stream << std::setw(indentation) << "";
  stream << "Equality Lagrangian:        " << std::setw(tabSpace) << performanceIndex.equalityLagrangian;
  stream << "Inequality Lagrangian:      " << std::setw(tabSpace) << performanceIndex.inequalityLagrangian;

  return stream;
}

}  // namespace ocs2
