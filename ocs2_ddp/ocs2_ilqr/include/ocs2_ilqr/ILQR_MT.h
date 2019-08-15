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

#ifndef ILQR_MT_OCS2_H_
#define ILQR_MT_OCS2_H_

#include <atomic>
#include <condition_variable>
#include <thread>

#include <ocs2_core/misc/SetThreadPriority.h>

#include "ocs2_ilqr/ILQR_BASE.h"

namespace ocs2 {

/**
 * This class implements multi-threaded ILQR algorithm.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ILQR_MT : public ILQR_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ILQR_BASE<STATE_DIM, INPUT_DIM>;

  using DIMENSIONS = typename BASE::DIMENSIONS;

  using typename BASE::size_array_t;
  using typename BASE::size_array2_t;
  using typename BASE::scalar_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_array2_t;
  using typename BASE::eigen_scalar_t;
  using typename BASE::eigen_scalar_array_t;
  using typename BASE::eigen_scalar_array2_t;
  using typename BASE::state_vector_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_array2_t;
  using typename BASE::input_vector_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_array2_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_state_matrix_array2_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_matrix_array2_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_matrix_array2_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_input_matrix_array_t;
  using typename BASE::state_input_matrix_array2_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint1_vector_array_t;
  using typename BASE::constraint1_vector_array2_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_state_matrix_array_t;
  using typename BASE::constraint1_state_matrix_array2_t;
  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_input_matrix_array_t;
  using typename BASE::constraint1_input_matrix_array2_t;
  using typename BASE::input_constraint1_matrix_t;
  using typename BASE::input_constraint1_matrix_array_t;
  using typename BASE::input_constraint1_matrix_array2_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::constraint2_vector_array_t;
  using typename BASE::constraint2_vector_array2_t;
  using typename BASE::constraint2_state_matrix_t;
  using typename BASE::constraint2_state_matrix_array_t;
  using typename BASE::constraint2_state_matrix_array2_t;

  using typename BASE::controller_t;
  using typename BASE::controller_array_t;
  using typename BASE::controller_ptr_array_t;
  using typename BASE::linear_controller_t;
  using typename BASE::linear_controller_array_t;
  using typename BASE::linear_controller_ptr_array_t;

  using typename BASE::controlled_system_base_t;
  using typename BASE::event_handler_t;
  using typename BASE::derivatives_base_t;
  using typename BASE::constraint_base_t;
  using typename BASE::cost_function_base_t;
  using typename BASE::operating_trajectories_base_t;

  /**
   * worker state enum
   */
  enum WORKER_STATE { IDLE = 0, LINE_SEARCH, APPROXIMATE_LQ, CALCULATE_CONTROLLER, SOLVE_RICCATI, SHUTDOWN };

  /**
   * Default constructor.
   */
  ILQR_MT() : BASE(), workerTask_(IDLE), subsystemProcessed_(0) {}

  /**
   * Constructor
   *
   * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of ILQR.
   * @param [in] settings: Structure containing the settings for the ILQR algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed logical dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  ILQR_MT(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
          const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
          const operating_trajectories_base_t* operatingTrajectoriesPtr, const ILQR_Settings& settings = ILQR_Settings(),
          std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr, const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  ~ILQR_MT();

  /**
   * Line search on the feedforwrd parts of the controller. It uses the following approach for line search:
   * The constraint TYPE-1 correction term is directly added through a user defined stepSize (defined in settings_.constraintStepSize_).
   * But the cost minimization term is optimized through a line-search strategy defined in ILQR settings.
   *
   * @param [in] computeISEs: Whether lineSearch needs to calculate ISEs indeces for type_1 and type-2 constraints.
   */
  void lineSearch(bool computeISEs) override;

  /**
   * Runs the initialization method for single thread ILQR.
   */
  void runInit() override;

  /**
   * Solves Riccati equations for all the partitions.
   *
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   *
   * @return average time step
   */
  scalar_t solveSequentialRiccatiEquations(const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
                                           const eigen_scalar_t& sFinal) override;

  /**
   * Runs a single iteration of single thread ILQR.
   */
  void runIteration() override;

  /**
   * Runs the exit method single thread ILQR.
   */
  void runExit() override;

 protected:
  /**
   * Computes the linearized dynamics for a particular time partition.
   *
   * @param [in] partitionIndex: Time partition index
   */
  void approximatePartitionLQ(size_t partitionIndex) override;

  /**
   * Finds the next node's uncompleted LQ approximation and executes approximateLQWorker.
   *
   * @param [in] threadId: Thread ID
   * @param [in] partitionIndex: Time partition index
   */
  void executeApproximatePartitionLQWorker(size_t threadId, size_t partitionIndex);

  /**
   * Computes the controller for a particular time partition
   *
   * @param partitionIndex: Time partition index
   */
  void calculatePartitionController(size_t partitionIndex) override;

  /**
   * Finds the next node's uncompleted CALCULATE_CONTROLLER task and executes calculateControllerWorker.
   *
   * @param [in] threadId: Thread ID
   * @param [in] partitionIndex: Time partition index
   */
  void executeCalculatePartitionController(size_t threadId, size_t partitionIndex);

  /**
   * Launches worker threads
   */
  void launchWorkerThreads();
  /**
   * Sets number of worker threads
   * @param [in] threadId: Thread ID
   */
  void threadWork(size_t threadId);

  /**
   * Execute line search worker on a thread with various learning rates and accept the result if it satisfies the step
   * size policy (defined in settings_.lsStepsizeGreedy_)
   *
   * @param [in] threadId
   */
  void executeLineSearchWorker(size_t threadId);

  /**
   * Distributes work
   */
  void distributeWork();

  /**
   * Solves Riccati equations for the partitions assigned to the given thread
   *
   * @param [in] threadId
   */
  void executeRiccatiSolver(size_t threadId);

  /**
   * a heuristic that generates a unique id for a process, such that we can manage the tasks.
   * Generates a unique identifiers for subsystem, task, iteration:
   * @param [in] iterateNo
   * @param [in] workerState
   * @param [in] subsystemId
   * @return size_t:
   */
  size_t generateUniqueProcessID(size_t iterateNo, int workerState, int subsystemId) {
    return (10e9 * (workerState + 1) + 10e6 * (subsystemId + 1) + iterateNo + 1);
  }

 private:
  // multi-threading helper variables
  std::vector<std::thread> workerThreads_;
  std::atomic_bool workersActive_;
  std::atomic_int workerTask_;
  std::atomic_int subsystemProcessed_;  // the subsystem the threads are currently working on

  std::mutex workerWakeUpMutex_;
  std::condition_variable workerWakeUpCondition_;
  std::mutex kCompletedMutex_;
  std::condition_variable kCompletedCondition_;
  std::mutex lineSearchResultMutex_;
  std::mutex alphaBestFoundMutex_;
  std::condition_variable alphaBestFoundCondition_;

  std::array<std::atomic_size_t, 100> kTaken_approx_;
  std::array<std::atomic_size_t, 100> kCompleted_approx_;
  std::array<std::atomic_size_t, 100> kTaken_ctrl_;
  std::array<std::atomic_size_t, 100> kCompleted_ctrl_;

  // parallel Riccati solver
  std::array<std::atomic_bool, 100> subsystemsDone_;
  std::array<std::atomic_bool, 100> subsystemsProcessing_;
  std::atomic_int numSubsystemsProcessed_;
  std::mutex riccatiSolverMutex_;
  std::mutex riccatiSolverBarrierMutex_;
  std::mutex riccatiSolverBarrierNotifyMutex_;
  std::mutex riccatiSolverDataMutex_;
  std::condition_variable riccatiSolverCompletedCondition_;
  std::vector<int> startingIndicesRiccatiWorker_;
  std::vector<int> endingIndicesRiccatiWorker_;

  size_t alphaMax_;
  size_t alphaExpBest_;
  size_t alphaExpMax_;
  scalar_t baselineTotalCost_;
  std::atomic_size_t alphaTaken_;
  std::atomic_bool alphaBestFound_;
  std::atomic_size_t lsWorkerCompleted_;
  std::vector<bool> alphaProcessed_;
};

}  // namespace ocs2

#include "implementation/ILQR_MT.h"

#endif /* ILQR_MT_H_ */
