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
  vector_t dualDirection = dual.cwiseProduct(slack + slackDirection);
  dualDirection.array() -= barrierParam;
  dualDirection.array() /= -slack.array();
  return dualDirection;
}

scalar_t fractionToBoundaryStepSize(const vector_t& v, const vector_t& dv, scalar_t marginRate) {
  assert(marginRate > 0.0);
  assert(marginRate <= 1.0);

  if (v.size() == 0) {
    return 1.0;
  }

  const vector_t invFractionToBoundary = (-1.0 / marginRate) * dv.cwiseQuotient(v);
  const auto alpha = invFractionToBoundary.maxCoeff();
  return alpha > 0.0 ? std::min(1.0 / alpha, 1.0) : 1.0;
}

namespace {
MultiplierCollection toMultiplierCollection(const multiple_shooting::ConstraintsSize constraintsSize, const vector_t& stateIneq) {
  MultiplierCollection multiplierCollection;
  size_t head = 0;
  for (const size_t size : constraintsSize.stateIneq) {
    multiplierCollection.stateIneq.emplace_back(0.0, stateIneq.segment(head, size));
    head += size;
  }
  return multiplierCollection;
}

MultiplierCollection toMultiplierCollection(const multiple_shooting::ConstraintsSize constraintsSize, const vector_t& stateIneq,
                                            const vector_t& stateInputIneq) {
  MultiplierCollection multiplierCollection = toMultiplierCollection(constraintsSize, stateIneq);
  size_t head = 0;
  for (const size_t size : constraintsSize.stateInputIneq) {
    multiplierCollection.stateInputIneq.emplace_back(0.0, stateInputIneq.segment(head, size));
    head += size;
  }
  return multiplierCollection;
}

vector_t extractLagrangian(const std::vector<Multiplier>& termsMultiplier) {
  size_t n = 0;
  std::for_each(termsMultiplier.begin(), termsMultiplier.end(), [&](const Multiplier& m) { n += m.lagrangian.size(); });

  vector_t vec(n);
  size_t head = 0;
  for (const auto& m : termsMultiplier) {
    vec.segment(head, m.lagrangian.size()) = m.lagrangian;
    head += m.lagrangian.size();
  }  // end of i loop

  return vec;
}
}  // namespace

DualSolution toDualSolution(const std::vector<AnnotatedTime>& time, const std::vector<multiple_shooting::ConstraintsSize>& constraintsSize,
                            const vector_array_t& stateIneq, const vector_array_t& stateInputIneq) {
  // Problem horizon
  const int N = static_cast<int>(time.size()) - 1;

  DualSolution dualSolution;
  dualSolution.timeTrajectory = toInterpolationTime(time);
  dualSolution.postEventIndices = toPostEventIndices(time);

  dualSolution.preJumps.reserve(dualSolution.postEventIndices.size());
  dualSolution.intermediates.reserve(time.size());

  for (int i = 0; i < N; ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent) {
      dualSolution.preJumps.emplace_back(toMultiplierCollection(constraintsSize[i], stateIneq[i]));
      dualSolution.intermediates.push_back(dualSolution.intermediates.back());  // no event at the initial node
    } else {
      dualSolution.intermediates.emplace_back(toMultiplierCollection(constraintsSize[i], stateIneq[i], stateInputIneq[i]));
    }
  }
  dualSolution.final = toMultiplierCollection(constraintsSize[N], stateIneq[N]);
  dualSolution.intermediates.push_back(dualSolution.intermediates.back());

  return dualSolution;
}

std::pair<vector_t, vector_t> fromMultiplierCollection(const MultiplierCollection& multiplierCollection) {
  return std::make_pair(extractLagrangian(multiplierCollection.stateIneq), extractLagrangian(multiplierCollection.stateInputIneq));
}

}  // namespace ipm
}  // namespace ocs2
