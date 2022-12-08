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

#pragma once

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace multiple_shooting {

/**
 * Coefficients to compute the Newton step of the Lagrange multiplier associated with the state-input equality constraint such that
 * dfdx * dx + dfdu * du + dfdcostate * dcostate + f
 */
struct ProjectionMultiplierCoefficients {
  matrix_t dfdx;
  matrix_t dfdu;
  matrix_t dfdcostate;
  vector_t f;

  /**
   * Computes the coefficients of the Lagrange multiplier associated with the state-input equality constraint.
   *
   * @param cost : The cost quadratic approximation.
   * @param dynamics : The dynamics linear approximation.
   * @param constraintProjection : The constraint projection.
   * @param pseudoInverse : Left pseudo-inverse of D^T of the state-input linearized equality constraint (C dx + D du + e = 0).
   */
  void compute(const ScalarFunctionQuadraticApproximation& cost, const VectorFunctionLinearApproximation& dynamics,
               const VectorFunctionLinearApproximation& constraintProjection, const matrix_t& pseudoInverse);
};

}  // namespace multiple_shooting
}  // namespace ocs2
