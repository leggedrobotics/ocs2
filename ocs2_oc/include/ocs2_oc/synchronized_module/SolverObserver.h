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

#include <functional>
#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/model_data/Metrics.h>

#include "ocs2_oc/oc_data/DualSolution.h"
#include "ocs2_oc/oc_data/ProblemMetrics.h"
#include "ocs2_oc/oc_problem/OptimalControlProblem.h"

namespace ocs2 {

/**
 * A solver observer module that extract the metrics and multiplier corresponding to the requested term.
 */
class SolverObserver {
 private:
  struct PrivateToken {};

 public:
  /** The public constructor which cannot be used for instantiation. For that, use the following factory methods:
   *     - SolverObserver::ConstraintTermObserver
   *     - SolverObserver::LagrangianTermObserver
   */
  explicit SolverObserver(PrivateToken token) {}

  /** The time of the metric that will be observed. */
  enum class Type {
    Final,
    PreJump,
    Intermediate,
  };

  /** Input arguments are time stamp and a const reference array of the constraint term's value. */
  using constraint_callback_t = std::function<void(const scalar_array_t&, const std::vector<std::reference_wrapper<const vector_t>>&)>;
  /** Input arguments are time stamp and a const reference array of the term's LagrangianMetrics. */
  using lagrangian_callback_t = std::function<void(const scalar_array_t&, const std::vector<LagrangianMetricsConstRef>&)>;
  /** Input arguments are time stamp and a const reference array of the term's Multiplier. */
  using multiplier_callback_t = std::function<void(const scalar_array_t&, const std::vector<MultiplierConstRef>&)>;

  /**
   * Sets the callback for processing extracted a constraint term.
   *
   * @param [in] type: It could be either at Final, PreJump, or Intermediate.
   * @param [in] termsName: The name of the term used to add it to OptimalControlProblem. Note that the name is case sensitive.
   * @param [in] constraintCallback: The callback for processing extracted constraint vector array. The callback should have the following
   * signature: void callback(const scalar_array_t& timeStamp, const std::vector<std::reference_wrapper<const vector_t>>& termMetrics). It
   * could any C-style function pointer, lambda, or std::function. For an example check constraint_callback_t.
   */
  template <class ConstraintCallbackType>
  static std::unique_ptr<SolverObserver> ConstraintTermObserver(Type type, const std::string& termName,
                                                                ConstraintCallbackType&& constraintCallback);

  /**
   * Sets the callback for processing extracted a term's LagrangianMetrics and/or Multiplier.
   *
   * @param [in] type: It could be either at Final, PreJump, or Intermediate.
   * @param [in] termsName: The name of the term used to add it to OptimalControlProblem. Note that the name is case sensitive.
   * @param [in] lagrangianCallback: The callback for processing extracted LagrangianMetrics array. The callback should have the following
   * signature: void callback(const scalar_array_t& timeStamp, std::vector<LagrangianMetricsConstRef>& termMetrics). It could any C-style
   * function pointer, lambda, or std::function. For an example check lagrangian_callback_t).
   * @param [in] multiplierCallback: The callback for processing extracted Multiplier array. The callback should have the following
   * signature: void callback(const scalar_array_t& timeStamp, const std::vector<MultiplierConstRef>& termMultiplier). It could any C-style
   * function pointer, lambda, or std::function. For an example check multiplier_callback_t.
   */
  template <class LagrangianCallbackType, class MultiplierCallbackType = multiplier_callback_t>
  static std::unique_ptr<SolverObserver> LagrangianTermObserver(Type type, const std::string& termName,
                                                                LagrangianCallbackType&& lagrangianCallback,
                                                                MultiplierCallbackType&& multiplierCallback = multiplier_callback_t());

 private:
  /**
   * Constructor.
   * @param [in] termsName: The name of the term used to add it to OptimalControlProblem.
   * @note The name is case sensitive.
   */
  template <class ConstraintCallbackType, class LagrangianCallbackType, class MultiplierCallbackType>
  SolverObserver(Type type, std::string termName, ConstraintCallbackType constraintCallback, LagrangianCallbackType lagrangianCallback,
                 MultiplierCallbackType multiplierCallback)
      : type_(type),
        termName_(std::move(termName)),
        constraintCallback_(std::move(constraintCallback)),
        lagrangianCallback_(std::move(lagrangianCallback)),
        multiplierCallback_(std::move(multiplierCallback)) {}

  /**
   * Extracts the constraint value associated to the requested term from the given problemMetrics.
   *
   * @param [in] ocp : The optimal control problem.
   * @param [in] primalSolution : The primal solution.
   * @param [in] problemMetrics: The metrics of current solution
   */
  void extractTermConstraint(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, const ProblemMetrics& problemMetrics);

  /**
   * Extracts the LagrangianMetrics associated to the requested term from the given problemMetrics.
   *
   * @param [in] ocp : The optimal control problem.
   * @param [in] primalSolution : The primal solution.
   * @param [in] problemMetrics: The metrics of current solution
   */
  void extractTermLagrangianMetrics(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution,
                                    const ProblemMetrics& problemMetrics);

  /**
   * Extracts the multipliers associated to the requested term from the given dualSolution.
   *
   * @param [in] ocp : The optimal control problem.
   * @param [in] dualSolution: The dual solution
   */
  void extractTermMultipliers(const OptimalControlProblem& ocp, const DualSolution& dualSolution);

  /**
   * Variables
   */
  const Type type_ = Type::Final;
  const std::string termName_ = "";
  constraint_callback_t constraintCallback_;
  lagrangian_callback_t lagrangianCallback_;
  multiplier_callback_t multiplierCallback_;

  /** SolverBase needs to call extractXXX private methods. */
  friend class SolverBase;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class ConstraintCallbackType>
std::unique_ptr<SolverObserver> SolverObserver::ConstraintTermObserver(Type type, const std::string& termName,
                                                                       ConstraintCallbackType&& constraintCallback) {
  return std::unique_ptr<SolverObserver>(new SolverObserver(type, termName, std::forward<ConstraintCallbackType>(constraintCallback),
                                                            lagrangian_callback_t(), multiplier_callback_t()));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <class LagrangianCallbackType, class MultiplierCallbackType>
std::unique_ptr<SolverObserver> SolverObserver::LagrangianTermObserver(Type type, const std::string& termName,
                                                                       LagrangianCallbackType&& lagrangianCallback,
                                                                       MultiplierCallbackType&& multiplierCallback) {
  return std::unique_ptr<SolverObserver>(new SolverObserver(type, termName, constraint_callback_t(),
                                                            std::forward<LagrangianCallbackType>(lagrangianCallback),
                                                            std::forward<MultiplierCallbackType>(multiplierCallback)));
}

}  // namespace ocs2
