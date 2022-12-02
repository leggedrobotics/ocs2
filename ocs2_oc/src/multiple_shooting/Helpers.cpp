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

#include "ocs2_oc/multiple_shooting/Helpers.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

namespace ocs2 {
namespace multiple_shooting {

void remapProjectedInput(const std::vector<VectorFunctionLinearApproximation>& constraintsProjection, const vector_array_t& deltaXSol,
                         vector_array_t& deltaUSol) {
  vector_t tmp;  // 1 temporary for re-use.
  for (int i = 0; i < deltaUSol.size(); ++i) {
    if (constraintsProjection[i].f.size() > 0) {
      tmp.noalias() = constraintsProjection[i].dfdu * deltaUSol[i];
      deltaUSol[i] = tmp + constraintsProjection[i].f;
      deltaUSol[i].noalias() += constraintsProjection[i].dfdx * deltaXSol[i];
    }
  }
}

void remapProjectedGain(const std::vector<VectorFunctionLinearApproximation>& constraintsProjection, matrix_array_t& KMatrices) {
  matrix_t tmp;  // 1 temporary for re-use.
  for (int i = 0; i < KMatrices.size(); ++i) {
    if (constraintsProjection[i].f.size() > 0) {
      tmp.noalias() = constraintsProjection[i].dfdu * KMatrices[i];
      KMatrices[i] = tmp + constraintsProjection[i].dfdx;
    }
  }
}

PrimalSolution toPrimalSolution(const std::vector<AnnotatedTime>& time, ModeSchedule&& modeSchedule, vector_array_t&& x,
                                vector_array_t&& u) {
  // Correct for missing inputs at PreEvents and the terminal time
  for (int i = 0; i < u.size(); ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
      u[i] = u[i - 1];
    }
  }
  // Repeat last input to make equal length vectors
  u.push_back(u.back());

  // Construct nominal time, state and input trajectories
  PrimalSolution primalSolution;
  primalSolution.timeTrajectory_ = toTime(time);
  primalSolution.postEventIndices_ = toPostEventIndices(time);
  primalSolution.stateTrajectory_ = std::move(x);
  primalSolution.inputTrajectory_ = std::move(u);
  primalSolution.modeSchedule_ = std::move(modeSchedule);
  primalSolution.controllerPtr_.reset(new FeedforwardController(primalSolution.timeTrajectory_, primalSolution.inputTrajectory_));
  return primalSolution;
}

PrimalSolution toPrimalSolution(const std::vector<AnnotatedTime>& time, ModeSchedule&& modeSchedule, vector_array_t&& x, vector_array_t&& u,
                                matrix_array_t&& KMatrices) {
  // Compute feedback, before x and u are moved to primal solution
  // see doc/LQR_full.pdf for detailed derivation for feedback terms
  vector_array_t uff = u;  // Copy and adapt in loop
  for (int i = 0; i < KMatrices.size(); ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
      uff[i] = uff[i - 1];
      KMatrices[i] = KMatrices[i - 1];
    } else {
      // Linear controller has convention u = uff + K * x;
      // We computed u = u'(t) + K (x - x'(t));
      // >> uff = u'(t) - K x'(t)
      uff[i].noalias() -= KMatrices[i] * x[i];
    }
  }
  // Copy last one to get correct length
  uff.push_back(uff.back());
  KMatrices.push_back(KMatrices.back());

  // Correct for missing inputs at PreEvents and the terminal time
  for (int i = 0; i < u.size(); ++i) {
    if (time[i].event == AnnotatedTime::Event::PreEvent && i > 0) {
      u[i] = u[i - 1];
    }
  }
  // Repeat last input to make equal length vectors
  u.push_back(u.back());

  // Construct nominal time, state and input trajectories
  PrimalSolution primalSolution;
  primalSolution.timeTrajectory_ = toTime(time);
  primalSolution.postEventIndices_ = toPostEventIndices(time);
  primalSolution.stateTrajectory_ = std::move(x);
  primalSolution.inputTrajectory_ = std::move(u);
  primalSolution.modeSchedule_ = std::move(modeSchedule);
  primalSolution.controllerPtr_.reset(new LinearController(primalSolution.timeTrajectory_, std::move(uff), std::move(KMatrices)));
  return primalSolution;
}

void ProjectionMultiplierCoefficients::extractProjectionMultiplierCoefficients(
    const VectorFunctionLinearApproximation& dynamics, const ScalarFunctionQuadraticApproximation& cost,
    const VectorFunctionLinearApproximation& constraintProjection, const matrix_t& pseudoInverse) {
  vector_t semiprojectedCost_dfdu = cost.dfdu;
  semiprojectedCost_dfdu.noalias() += cost.dfduu * constraintProjection.f;

  matrix_t semiprojectedCost_dfdux = cost.dfdux;
  semiprojectedCost_dfdux.noalias() += cost.dfduu * constraintProjection.dfdx;

  const matrix_t semiprojectedCost_dfduu = cost.dfduu * constraintProjection.dfdu;

  this->dfdx.noalias() = -pseudoInverse * semiprojectedCost_dfdux;
  this->dfdu.noalias() = -pseudoInverse * semiprojectedCost_dfduu;
  this->dfdcostate.noalias() = -pseudoInverse * dynamics.dfdu.transpose();
  this->f.noalias() = -pseudoInverse * semiprojectedCost_dfdu;
}

}  // namespace multiple_shooting
}  // namespace ocs2
