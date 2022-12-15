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

#include "ocs2_ipm/IpmHelpers.h"

#include <cassert>

namespace ocs2 {
namespace ipm {

void condenseIneqConstraints(scalar_t barrierParam, const vector_t& slack, const vector_t& dual,
                             const VectorFunctionLinearApproximation& ineqConstraint, ScalarFunctionQuadraticApproximation& lagrangian) {
  assert(barrierParam > 0.0);

  const size_t nc = ineqConstraint.f.size();
  const size_t nu = ineqConstraint.dfdu.cols();

  if (nc == 0) {
    return;
  }

  // dual feasibilities
  lagrangian.dfdx.noalias() -= ineqConstraint.dfdx.transpose() * dual;
  if (ineqConstraint.dfdu.cols() > 0) {
    lagrangian.dfdu.noalias() -= ineqConstraint.dfdu.transpose() * dual;
  }

  // coefficients for condensing
  const vector_t condensingLinearCoeff = (dual.array() * ineqConstraint.f.array() - barrierParam) / slack.array();
  const vector_t condensingQuadraticCoeff = dual.cwiseQuotient(slack);

  // condensing
  lagrangian.dfdx.noalias() += ineqConstraint.dfdx.transpose() * condensingLinearCoeff;
  const matrix_t condensingQuadraticCoeff_dfdx = condensingQuadraticCoeff.asDiagonal() * ineqConstraint.dfdx;
  lagrangian.dfdxx.noalias() += ineqConstraint.dfdx.transpose() * condensingQuadraticCoeff_dfdx;

  if (nu > 0) {
    lagrangian.dfdu.noalias() += ineqConstraint.dfdu.transpose() * condensingLinearCoeff;
    const matrix_t condensingQuadraticCoeff_dfdu = condensingQuadraticCoeff.asDiagonal() * ineqConstraint.dfdu;
    lagrangian.dfduu.noalias() += ineqConstraint.dfdu.transpose() * condensingQuadraticCoeff_dfdu;
    lagrangian.dfdux.noalias() += ineqConstraint.dfdu.transpose() * condensingQuadraticCoeff_dfdx;
  }
}

vector_t retrieveSlackDirection(const VectorFunctionLinearApproximation& stateInputIneqConstraints, const vector_t& dx, const vector_t& du,
                                scalar_t barrierParam, const vector_t& slackStateInputIneq) {
  assert(barrierParam > 0.0);
  if (stateInputIneqConstraints.f.size() == 0) {
    return vector_t();
  }

  vector_t slackDirection = stateInputIneqConstraints.f - slackStateInputIneq;
  slackDirection.noalias() += stateInputIneqConstraints.dfdx * dx;
  slackDirection.noalias() += stateInputIneqConstraints.dfdu * du;
  return slackDirection;
}

vector_t retrieveSlackDirection(const VectorFunctionLinearApproximation& stateIneqConstraints, const vector_t& dx, scalar_t barrierParam,
                                const vector_t& slackStateIneq) {
  assert(barrierParam > 0.0);
  if (stateIneqConstraints.f.size() == 0) {
    return vector_t();
  }

  vector_t slackDirection = stateIneqConstraints.f - slackStateIneq;
  slackDirection.noalias() += stateIneqConstraints.dfdx * dx;
  return slackDirection;
}

vector_t retrieveDualDirection(scalar_t barrierParam, const vector_t& slack, const vector_t& dual, const vector_t& slackDirection) {
  assert(barrierParam > 0.0);
  vector_t dualDirection(slackDirection.size());
  dualDirection.array() = -(dual.array() * slackDirection.array() + (slack.array() * dual.array() - barrierParam)) / slack.array();
  return dualDirection;
}

scalar_t fractionToBoundaryStepSize(const vector_t& v, const vector_t& dv, scalar_t marginRate) {
  assert(marginRate > 0.0);
  assert(marginRate <= 1.0);

  if (v.size() == 0) {
    return 1.0;
  }

  scalar_t minFractionToBoundary = 1.0;
  vector_t fractionToBoundary = -marginRate * v.cwiseQuotient(dv);
  for (int i = 0; i < fractionToBoundary.size(); ++i) {
    if (fractionToBoundary[i] <= 0.0) {
      fractionToBoundary[i] = 1.0;
    }
  }
  return std::min(1.0, fractionToBoundary.minCoeff());
}

}  // namespace ipm
}  // namespace ocs2
