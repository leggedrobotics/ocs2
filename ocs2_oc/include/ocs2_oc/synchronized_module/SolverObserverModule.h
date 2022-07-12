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
class SolverObserverModule {
 public:
  /**
   * Constructor.
   * @param [in] termsName: The name of the term used to add it to OptimalControlProblem.
   * @note The name is case sensitive.
   */
  explicit SolverObserverModule(std::string termsName) : termsName_(std::move(termsName)) {}

  virtual ~SolverObserverModule() = default;
  virtual SolverObserverModule* clone() const { return new SolverObserverModule(*this); }

  /**
   * Sets the callback for processing extracted metrics array.
   * The callback should have the following signature:
   * void callback(const scalar_array_t& timeTrajectory, const std::vector<LagrangianMetricsConstRef>& termMetrics)
   */
  template <class CallbackType>
  void setMetricsCallback(CallbackType&& callback) {
    metricsCallback_ = std::forward<CallbackType>(callback);
  }

  /**
   * Sets the callback for processing extracted multiplier array.
   * The callback should have the following signature:
   * void callback(const scalar_array_t& timeTrajectory, const std::vector<MultiplierConstRef>& termMultiplierArray)
   */
  template <class CallbackType>
  void setMultiplierCallback(CallbackType&& callback) {
    multiplierCallback_ = std::forward<CallbackType>(callback);
  }

  /**
   * Extracts the metrics associated to the requested term from the given problemMetrics.
   *
   * @param [in] ocp : The optimal control problem.
   * @param [in] primalSolution : The primal solution.
   * @param [in] problemMetrics: The metrics of current solution
   */
  void extractTermMetrics(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, const ProblemMetrics& problemMetrics);

  /**
   * Extracts the multipliers associated to the requested term from the given dualSolution.
   *
   * @param [in] ocp : The optimal control problem.
   * @param [in] dualSolution: The dual solution
   */
  void extractTermMultipliers(const OptimalControlProblem& ocp, const DualSolution& dualSolution);

 protected:
  SolverObserverModule(const SolverObserverModule& other)
      : termsName_(other.termsName_), metricsCallback_(other.metricsCallback_), multiplierCallback_(other.multiplierCallback_) {}

  const std::string termsName_;

 private:
  std::vector<LagrangianMetricsConstRef> termMetricsArray_{};
  std::vector<MultiplierConstRef> termMultiplierArray_{};

  std::function<void(const scalar_array_t&, const std::vector<LagrangianMetricsConstRef>&)> metricsCallback_;
  std::function<void(const scalar_array_t&, const std::vector<MultiplierConstRef>&)> multiplierCallback_;
};

}  // namespace ocs2
