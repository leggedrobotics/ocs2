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

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

namespace ocs2 {
namespace continuous_time_lqr {

struct solution {
  /** Feedback gains \delta u = K \delta x */
  matrix_t feedbackGains;
  /** Quadratic value function J = 0.5 \delta x S \delta x */
  matrix_t valueFunction;
};

struct Settings {
  /** CARE iterations are stopped if the norm of the update step is below the specified tolerance */
  scalar_t tolerance = 1e-12;
  /** Maximum number of iterations */
  size_t maxIter = 100;
  /** Check numerical characteristic of the linear quadratic approximation */
  bool checkNumericalCharacteristics = true;
};

/**
 * This function solves the infinite-horizon continuous time LQR problem for unconstrained systems.
 *
 * The problem is solved based on the linear quadratic approximation at a given time, state, and input.
 * It is assumed that the provided linearization point is an equilibrium of the system and the minimum of the cost function.
 *
 * The solution is provided in terms the feedback gain \delta u = K \delta x, and value function matrix S s.t. J = 0.5 \delta x S \delta x
 *
 *  Implements "Algorithm 13.5.6. The Matrix Sign Function Algorithm for the CARE" from the following reference:
 *      CHAPTER 13 - NUMERICAL SOLUTIONS AND CONDITIONING OF ALGEBRAIC RICCATI EQUATIONS,
 *      Editor(s): BISWA NATH DATTA,
 *      Numerical Methods for Linear Control Systems,
 *      Academic Press, 2004, Pages 519-599, ISBN 9780122035906, https://doi.org/10.1016/B978-012203590-6/50017-3.
 *
 * @param systemDynamics : The system dynamics
 * @param problem : The optimal control problem
 * @param time : time around which the LQ approximation is made.
 * @param state : state around which the LQ approximation is made.
 * @param input : input around which the LQ approximation is made.
 * @param settings : algorithm settings.
 * @return {FeedbackGains K, Value function S}
 */
solution solve(OptimalControlProblem& problem, scalar_t time, const vector_t& state, const vector_t& input,
               const Settings& settings = Settings());

}  // namespace continuous_time_lqr
}  // namespace ocs2
