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
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

namespace ocs2 {

//  struct IntermediateMultipliersRef {
//    IntermediateMultipliersRef(const vector_array_t& stateEqArg, const vector_array_t& stateIneqArg, const vector_array_t&
//    stateInputIneqArg)
//        : stateEq(stateEqArg), stateIneq(stateIneqArg), stateInputIneq(stateInputIneqArg) {}
//
//    // state equality
//    const vector_array_t& stateEq;
//    // state inequality
//    const vector_array_t& stateIneq;
//    // state-input inequality
//    const vector_array_t& stateInputIneq;
//  };

//  struct EventMultipliersRef {
//    EventMultipliersRef(const vector_array_t& stateEqArg, const vector_array_t& stateIneqArg)
//        : stateEq(stateEqArg), stateIneq(stateIneqArg) {}
//
//    // state equality
//    const vector_array_t& stateEq;
//    // state inequality
//    const vector_array_t& stateIneq;
//  };

struct IntermediateMetrics {
  //  using value_t = std::pair<vector_t, scalar_t>;

  // cost
  scalar_t cost;
  // state equality
  //  std::vector<value_t> stateEqConstraint;
  scalar_t stateEqPenalty;
  // state-input equality
  vector_t stateInputEqConstraint;
  // state inequality
  //    std::vector<value_t>  stateIneqConstraint;
  scalar_t stateIneqPenalty;
  // state-input inequality
  //    std::vector<value_t> stateInputIneqConstraint;
  scalar_t stateInputIneqPenalty;
};

struct EventMetrics {
  //  using value_t = std::pair<vector_t, scalar_t>;

  // cost
  scalar_t cost;
  // state equality
  //    std::vector<value_t> stateEqConstraint;
  scalar_t stateEqPenalty;
  // state inequality
  //    std::vector<value_t> stateIneqConstraint;
  scalar_t stateIneqPenalty;
};

struct Metrics {
  EventMetrics final;
  std::vector<EventMetrics> events;
  std::vector<IntermediateMetrics> intermediates;
};

inline void swap(EventMetrics& lhs, EventMetrics& rhs) {
  std::swap(lhs.cost, rhs.cost);
  //    lhs.stateEqConstraint.swap(rhs.stateEqConstraint);
  std::swap(lhs.stateEqPenalty, rhs.stateEqPenalty);
  //    lhs.stateIneqConstraint.swap(rhs.stateIneqConstraint);
  std::swap(lhs.stateIneqPenalty, rhs.stateIneqPenalty);
}

inline void swap(Metrics& lhs, Metrics& rhs) {
  swap(lhs.final, rhs.final);
  lhs.events.swap(rhs.events);
  lhs.intermediates.swap(rhs.intermediates);
}

/**
 * Compute the intermediate-time metrics (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->request(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x, u)
 */
IntermediateMetrics computeIntermediateMetrics(OptimalControlProblem& problem, scalar_t t, const vector_t& x, const vector_t& u);

/**
 * Compute the event-time metrics (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->requestPreJump(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x)
 */
EventMetrics computeEventMetrics(OptimalControlProblem& problem, scalar_t t, const vector_t& x);

/**
 * Compute the final-time metrics (i.e. cost, softConstraints, and constraints).
 *
 * @note It is assumed that the precomputation request is already made.
 * problem.preComputationPtr->requestFinal(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x)
 */
EventMetrics computeFinalMetrics(OptimalControlProblem& problem, scalar_t t, const vector_t& x);

}  // namespace ocs2
