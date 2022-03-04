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

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/ControllerBase.h>

#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/oc_solver/PerformanceIndex.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 */
class SolverBase {
 public:
  /**
   * Constructor.
   */
  SolverBase();

  /**
   * Default destructor.
   */
  virtual ~SolverBase() = default;

  /**
   * Resets the class to its state after construction.
   */
  virtual void reset() = 0;

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, and final time.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   */
  void run(scalar_t initTime, const vector_t& initState, scalar_t finalTime);

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, final time, and
   * initial controller.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The time partitioning.
   * @param [in] externalControllerPtr: A pointer to the initial control policies. If you want to use the control policy which was designed
   * by the previous call of the "run" routine, you should either pass nullptr or use the other run method. In either cases, two scenarios
   * are possible: either the internal controller is already available (such as the MPC case where the warm starting option is set true) or
   * the internal controller is empty in which instead of performing a rollout the operating trajectories will be used.
   */
  void run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr);

  /**
   * Sets the ReferenceManager which manages both ModeSchedule and TargetTrajectories. This module updates before SynchronizedModules.
   */
  void setReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr) {
    if (referenceManagerPtr == nullptr) {
      throw std::runtime_error("[SolverBase] ReferenceManager pointer cannot be a nullptr!");
    }
    referenceManagerPtr_ = std::move(referenceManagerPtr);
  }

  /*
   * Gets the ReferenceManager which manages both ModeSchedule and TargetTrajectories.
   */
  ReferenceManagerInterface& getReferenceManager() { return *referenceManagerPtr_; }
  const ReferenceManagerInterface& getReferenceManager() const { return *referenceManagerPtr_; }

  /**
   * Sets all modules that need to be synchronized with the solver. Each module is updated once before and once after solving the problem
   */
  void setSynchronizedModules(const std::vector<std::shared_ptr<SolverSynchronizedModule>>& synchronizedModules) {
    synchronizedModules_ = synchronizedModules;
  }

  /**
   * Adds one module to the vector of modules that need to be synchronized with the solver. Each module is updated once before and once
   * after solving the problem
   */
  void addSynchronizedModule(std::shared_ptr<SolverSynchronizedModule> synchronizedModule) {
    synchronizedModules_.push_back(std::move(synchronizedModule));
  }

  /**
   * Returns the cost, merit function and ISEs of constraints for the latest optimized trajectory.
   *
   * @return PerformanceIndex of the last optimized trajectory.
   */
  virtual const PerformanceIndex& getPerformanceIndeces() const = 0;

  /**
   * Gets number of iterations.
   *
   * @return Number of iterations.
   */
  virtual size_t getNumIterations() const = 0;

  /**
   * Returns the history of the cost, merit function and ISEs of constraints for the iterations os the optimized trajectory.
   *
   * @return An array of PerformanceIndices.
   */
  virtual const std::vector<PerformanceIndex>& getIterationsLog() const = 0;

  /**
   * Gets final time of optimization
   *
   * @return finalTime
   */
  virtual scalar_t getFinalTime() const = 0;

  /**
   * @brief Returns the optimized policy data.
   *
   * @param [in] finalTime: The final time.
   * @param [out] primalSolutionPtr: The primal problem's solution.
   */
  virtual void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const = 0;

  /**
   * @brief Returns the optimized policy data.
   *
   * @param [in] finalTime: The final time.
   * @return: The primal problem's solution.
   */
  PrimalSolution primalSolution(scalar_t finalTime) const;

  /**
   * Calculates the value function quadratic approximation at the given time and state.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @return The quadratic approximation of the value function at the requested time and state.
   */
  virtual ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const = 0;

  /**
   * Calculates the Hamiltonian quadratic approximation at the given time, state and input.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @param [in] input: The inquiry input.
   * @return The quadratic approximation of the Hamiltonian at the requested time, state and input.
   */
  virtual ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) = 0;

  /**
   * Calculates the Lagrange multiplier of the state-input equality constraints at the given time and state.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @return The Lagrange multiplier of the state-input equality constraints.
   */
  virtual vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const = 0;

  /**
   * Gets benchmarking information.
   */
  virtual std::string getBenchmarkingInfo() const { return {}; }

  /**
   * Prints to output.
   *
   * @param [in] input text.
   */
  void printString(const std::string& text) const;

 private:
  virtual void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) = 0;

  virtual void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) = 0;

  void preRun(scalar_t initTime, const vector_t& initState, scalar_t finalTime);

  void postRun();

  /***********
   * Variables
   ***********/
  mutable std::mutex outputDisplayGuardMutex_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;  // this pointer cannot be nullptr
  std::vector<std::shared_ptr<SolverSynchronizedModule>> synchronizedModules_;
};

}  // namespace ocs2
