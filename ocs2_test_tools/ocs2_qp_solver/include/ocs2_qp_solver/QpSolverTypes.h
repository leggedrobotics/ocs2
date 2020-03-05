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

//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace ocs2 {
namespace qp_solver {

/**
 * Defines the quadratic approximation f(x,u) = 1/2 dx' dfdxx dx + du' dfdux dx + 1/2 du' dfduu du + dfdx' dx + dfdu' du + f
 */
struct ScalarFunctionQuadraticApproximation {
  /** Second derivative w.r.t state */
  Eigen::MatrixXd dfdxx;
  /** Second derivative w.r.t input (lhs) and state (rhs) */
  Eigen::MatrixXd dfdux;
  /** Second derivative w.r.t input */
  Eigen::MatrixXd dfduu;
  /** First derivative w.r.t state */
  Eigen::VectorXd dfdx;
  /** First derivative w.r.t input */
  Eigen::VectorXd dfdu;
  /** Constant term */
  double f = 0.;
};

/**
 * Defines the linear model of a vector function f(x,u) = dfdx * dx + dfdu * du + df
 */
struct VectorFunctionLinearApproximation {
  /** Derivative w.r.t state */
  Eigen::MatrixXd dfdx;
  /** Derivative w.r.t input */
  Eigen::MatrixXd dfdu;
  /** Constant term */
  Eigen::VectorXd f;
};

/** Defines the quadratic cost and  linear dynamics at a give stage */
struct LinearQuadraticStage {
  /** Quadratic approximation of the cost */
  ScalarFunctionQuadraticApproximation cost;
  /** Linear approximation of the dynamics */
  VectorFunctionLinearApproximation dynamics;

  LinearQuadraticStage() = default;
  LinearQuadraticStage(ScalarFunctionQuadraticApproximation c, VectorFunctionLinearApproximation d)
      : cost(std::move(c)), dynamics(std::move(d)) {}
};

}  // namespace qp_solver
}  // namespace ocs2
