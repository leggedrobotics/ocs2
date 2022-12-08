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

#include <gtest/gtest.h>

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_oc/multiple_shooting/ProjectionMultiplierCoefficients.h>

#include "ocs2_oc/test/testProblemsGeneration.h"

using namespace ocs2;

TEST(testMultipleShootingHelpers, testProjectionMultiplierCoefficients) {
  constexpr size_t stateDim = 30;
  constexpr size_t inputDim = 20;
  constexpr size_t constraintDim = 10;

  const auto cost = getRandomCost(stateDim, inputDim);
  const auto dynamics = getRandomDynamics(stateDim, inputDim);
  const auto constraint = getRandomConstraints(stateDim, inputDim, constraintDim);

  auto result = LinearAlgebra::qrConstraintProjection(constraint);
  const auto projection = std::move(result.first);
  const auto pseudoInverse = std::move(result.second);

  multiple_shooting::ProjectionMultiplierCoefficients projectionMultiplierCoefficients;
  projectionMultiplierCoefficients.compute(cost, dynamics, projection, pseudoInverse);

  const matrix_t dfdx = -pseudoInverse * (cost.dfdux + cost.dfduu * projection.dfdx);
  const matrix_t dfdu = -pseudoInverse * (cost.dfduu * projection.dfdu);
  const matrix_t dfdcostate = -pseudoInverse * dynamics.dfdu.transpose();
  const vector_t f = -pseudoInverse * (cost.dfdu + cost.dfduu * projection.f);

  ASSERT_TRUE(projectionMultiplierCoefficients.dfdx.isApprox(dfdx));
  ASSERT_TRUE(projectionMultiplierCoefficients.dfdu.isApprox(dfdu));
  ASSERT_TRUE(projectionMultiplierCoefficients.dfdcostate.isApprox(dfdcostate));
  ASSERT_TRUE(projectionMultiplierCoefficients.f.isApprox(f));
}
