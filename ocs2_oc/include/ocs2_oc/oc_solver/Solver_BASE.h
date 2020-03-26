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

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <unsupported/Eigen/MatrixFunctions>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <numeric>
#include <type_traits>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/logic/ModeSchedule.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/Numerics.h>

#include "ocs2_oc/oc_data/PrimalSolution.h"
#include "ocs2_oc/oc_solver/ModeScheduleManager.h"
#include "ocs2_oc/oc_solver/PerformanceIndex.h"
#include "ocs2_oc/oc_solver/SolverSynchronizedModule.h"

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class Solver_BASE {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;

  using size_array_t = typename DIMENSIONS::size_array_t;
  using size_array2_t = typename DIMENSIONS::size_array2_t;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using scalar_array2_t = typename DIMENSIONS::scalar_array2_t;
  using scalar_array3_t = typename DIMENSIONS::scalar_array3_t;
  using eigen_scalar_t = typename DIMENSIONS::eigen_scalar_t;
  using eigen_scalar_array_t = typename DIMENSIONS::eigen_scalar_array_t;
  using eigen_scalar_array2_t = typename DIMENSIONS::eigen_scalar_array2_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using state_vector_array2_t = typename DIMENSIONS::state_vector_array2_t;
  using state_vector_array3_t = typename DIMENSIONS::state_vector_array3_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_vector_array2_t = typename DIMENSIONS::input_vector_array2_t;
  using input_vector_array3_t = typename DIMENSIONS::input_vector_array3_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using input_state_matrix_array2_t = typename DIMENSIONS::input_state_matrix_array2_t;
  using input_state_matrix_array3_t = typename DIMENSIONS::input_state_matrix_array3_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using state_matrix_array2_t = typename DIMENSIONS::state_matrix_array2_t;
  using state_matrix_array3_t = typename DIMENSIONS::state_matrix_array3_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using input_matrix_array2_t = typename DIMENSIONS::input_matrix_array2_t;
  using input_matrix_array3_t = typename DIMENSIONS::input_matrix_array3_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
  using state_input_matrix_array2_t = typename DIMENSIONS::state_input_matrix_array2_t;
  using state_input_matrix_array3_t = typename DIMENSIONS::state_input_matrix_array3_t;
  using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
  using constraint1_vector_array_t = typename DIMENSIONS::constraint1_vector_array_t;
  using constraint1_vector_array2_t = typename DIMENSIONS::constraint1_vector_array2_t;
  using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_state_matrix_array_t = typename DIMENSIONS::constraint1_state_matrix_array_t;
  using constraint1_state_matrix_array2_t = typename DIMENSIONS::constraint1_state_matrix_array2_t;
  using constraint1_input_matrix_t = typename DIMENSIONS::constraint1_input_matrix_t;
  using constraint1_input_matrix_array_t = typename DIMENSIONS::constraint1_input_matrix_array_t;
  using constraint1_input_matrix_array2_t = typename DIMENSIONS::constraint1_input_matrix_array2_t;
  using input_constraint1_matrix_t = typename DIMENSIONS::input_constraint1_matrix_t;
  using input_constraint1_matrix_array_t = typename DIMENSIONS::input_constraint1_matrix_array_t;
  using input_constraint1_matrix_array2_t = typename DIMENSIONS::input_constraint1_matrix_array2_t;
  using constraint2_vector_t = typename DIMENSIONS::constraint2_vector_t;
  using constraint2_vector_array_t = typename DIMENSIONS::constraint2_vector_array_t;
  using constraint2_vector_array2_t = typename DIMENSIONS::constraint2_vector_array2_t;
  using constraint2_state_matrix_t = typename DIMENSIONS::constraint2_state_matrix_t;
  using constraint2_state_matrix_array_t = typename DIMENSIONS::constraint2_state_matrix_array_t;
  using constraint2_state_matrix_array2_t = typename DIMENSIONS::constraint2_state_matrix_array2_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_matrix_t = typename DIMENSIONS::dynamic_matrix_t;
  using dynamic_vector_array_t = typename DIMENSIONS::dynamic_vector_array_t;
  using dynamic_matrix_array2_t = typename DIMENSIONS::dynamic_matrix_array2_t;
  using dynamic_input_matrix_t = typename DIMENSIONS::dynamic_input_matrix_t;

  using primal_solution_t = PrimalSolution<STATE_DIM, INPUT_DIM>;

  using controller_t = ControllerBase<STATE_DIM, INPUT_DIM>;
  using controller_array_t = typename controller_t::array_t;
  using controller_ptr_array_t = std::vector<controller_t*>;
  using controller_const_ptr_array_t = std::vector<const controller_t*>;
  using feedforward_controller_t = FeedforwardController<STATE_DIM, INPUT_DIM>;

  using synchronized_module_t = SolverSynchronizedModule<STATE_DIM, INPUT_DIM>;
  using synchronized_module_ptr_array_t = std::vector<std::shared_ptr<synchronized_module_t>>;
  using mode_schedule_manager_ptr_t = std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>>;

  /**
   * Constructor.
   */
  Solver_BASE() = default;

  /**
   * Default destructor.
   */
  virtual ~Solver_BASE() = default;

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
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   */
  void run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes);

  /**
   * The main routine of solver which runs the optimizer for a given initial state, initial time, final time, and
   * initial controller.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The time partitioning.
   * @param [in] controllersPtrStock: Array of pointers to the initial control policies. If you want to use the control
   * policy which was designed by the previous call of the "run" routine, you should pass an empty array. In the this case, two scenarios
   * are possible: either the internal controller is already set (such as the MPC case where the warm starting option is set true) or the
   * internal controller is empty in which instead of performing a rollout the operating trajectories will be used.
   */
  void run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
           const controller_ptr_array_t& controllersPtrStock);

  /**
   * Set mode schedule manager. This module is updated once before and once after solving the problem.
   */
  void setModeScheduleManager(mode_schedule_manager_ptr_t modeScheduleManager) { modeScheduleManager_ = std::move(modeScheduleManager); };

  /**
   * Set all modules that need to be synchronized with the solver. Each module is updated once before and once after solving the problem
   */
  void setSynchronizedModules(const synchronized_module_ptr_array_t& synchronizedModules) { synchronizedModules_ = synchronizedModules; };

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
   * Returns the partitioning times
   *
   * @return partitioning times
   */
  virtual const scalar_array_t& getPartitioningTimes() const = 0;

  /**
   * Gets the cost function desired trajectories.
   *
   * @param [out] costDesiredTrajectories: A pointer to the cost function desired trajectories
   */
  const CostDesiredTrajectories& getCostDesiredTrajectories() const { return costDesiredTrajectories_; };

  /**
   * Sets the cost function desired trajectories.
   *
   * @param [in] costDesiredTrajectories: The cost function desired trajectories
   */
  void setCostDesiredTrajectories(const CostDesiredTrajectories& costDesiredTrajectories) {
    costDesiredTrajectories_ = costDesiredTrajectories;
  };

  /**
   * Swaps the cost function desired trajectories.
   *
   * @param [in] costDesiredTrajectories: The cost function desired trajectories
   */
  void swapCostDesiredTrajectories(CostDesiredTrajectories& costDesiredTrajectories) {
    costDesiredTrajectories_.swap(costDesiredTrajectories);
  };

  /** gets mode schedule */
  const ModeSchedule& getModeSchedule() const { return modeSchedule_; }

  /**
   * @brief Returns the optimized policy data.
   *
   * @param [in] finalTime: The final time.
   * @param [out] primalSolutionPtr: The primal problem's solution.
   */
  virtual void getPrimalSolution(scalar_t finalTime, primal_solution_t* primalSolutionPtr) const = 0;

  /**
   * @brief Returns the optimized policy data.
   *
   * @param [in] finalTime: The final time.
   * @return: The primal problem's solution.
   */
  primal_solution_t primalSolution(scalar_t finalTime) const;

  /**
   * Calculates the value function at the given time and state.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @return value at the inquiry time and state.
   */
  virtual scalar_t getValueFunction(scalar_t time, const state_vector_t& state) const = 0;

  /**
   * Calculates the value function state derivative at the given time and state.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @param [out] Vx: value function at the inquiry time and state.
   */
  virtual void getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state, state_vector_t& Vx) const = 0;

  /**
   * Calculates the Lagrange multiplier of the state-input equality constraints at the given time and state.
   *
   * @param [in] time: The inquiry time
   * @param [in] state: The inquiry state.
   * @param [out] nu: The Lagrange multiplier of the state-input equality constraints.
   */
  virtual void getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state, dynamic_vector_t& nu) const = 0;

  /**
   * Rewinds optimizer internal variables.
   *
   * @param [in] firstIndex: The index which we want to rewind to.
   */
  virtual void rewindOptimizer(size_t firstIndex) = 0;

  /**
   * Get rewind counter.
   *
   * @return Number of partition rewinds since construction of the class.
   */
  virtual const unsigned long long int& getRewindCounter() const = 0;

  /**
   * Prints to output.
   *
   * @param [in] input text.
   */
  void printString(const std::string& text);

 private:
  virtual void runImpl(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) = 0;

  virtual void runImpl(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
                       const controller_ptr_array_t& controllersPtrStock) = 0;

  void preRun(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime);

  void postRun();

  std::mutex outputDisplayGuardMutex_;
  CostDesiredTrajectories costDesiredTrajectories_;
  mode_schedule_manager_ptr_t modeScheduleManager_;
  synchronized_module_ptr_array_t synchronizedModules_;
  ModeSchedule modeSchedule_;
};

}  // namespace ocs2

#include "implementation/Solver_BASE.h"
