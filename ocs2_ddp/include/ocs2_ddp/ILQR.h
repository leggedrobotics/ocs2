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

#include <ocs2_ddp/DDP_BASE.h>

#include <ocs2_ddp/ILQR_Settings.h>

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread ILQR.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ILQR : public DDP_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = DDP_BASE<STATE_DIM, INPUT_DIM>;

  using typename BASE::constraint1_input_matrix_array2_t;
  using typename BASE::constraint1_input_matrix_array_t;
  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_array2_t;
  using typename BASE::constraint1_state_matrix_array_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_array2_t;
  using typename BASE::constraint1_vector_array_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_state_matrix_array2_t;
  using typename BASE::constraint2_state_matrix_array_t;
  using typename BASE::constraint2_state_matrix_t;
  using typename BASE::constraint2_vector_array2_t;
  using typename BASE::constraint2_vector_array_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::DIMENSIONS;
  using typename BASE::dynamic_input_matrix_t;
  using typename BASE::dynamic_matrix_array2_t;
  using typename BASE::dynamic_matrix_t;
  using typename BASE::dynamic_vector_array_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::eigen_scalar_array2_t;
  using typename BASE::eigen_scalar_array_t;
  using typename BASE::eigen_scalar_t;
  using typename BASE::input_constraint1_matrix_array2_t;
  using typename BASE::input_constraint1_matrix_array_t;
  using typename BASE::input_constraint1_matrix_t;
  using typename BASE::input_matrix_array2_t;
  using typename BASE::input_matrix_array3_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_array2_t;
  using typename BASE::input_state_matrix_array3_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array2_t;
  using typename BASE::input_vector_array3_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array2_t;
  using typename BASE::scalar_array3_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array2_t;
  using typename BASE::size_array_t;
  using typename BASE::state_input_matrix_array2_t;
  using typename BASE::state_input_matrix_array3_t;
  using typename BASE::state_input_matrix_array_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_array2_t;
  using typename BASE::state_matrix_array3_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array2_t;
  using typename BASE::state_vector_array3_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using typename BASE::controller_array_t;
  using typename BASE::controller_ptr_array_t;
  using typename BASE::controller_t;
  using typename BASE::linear_controller_array_t;
  using typename BASE::linear_controller_ptr_array_t;
  using typename BASE::linear_controller_t;

  using typename BASE::constraint_base_t;
  using typename BASE::cost_function_base_t;
  using typename BASE::derivatives_base_t;
  using typename BASE::event_handler_t;
  using typename BASE::linear_quadratic_approximator_t;
  using typename BASE::operating_trajectorie_rollout_t;
  using typename BASE::operating_trajectories_base_t;
  using typename BASE::penalty_base_t;
  using typename BASE::rollout_base_t;

  //	/**
  //	 * class for collecting ILQR data
  //	 */
  //	template <size_t OTHER_STATE_DIM, size_t OTHER_INPUT_DIM>
  //	friend class SLQ_DataCollector;

  /**
   * Default constructor.
   */
  ILQR() = default;

  /**
   * Constructor
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of ILQR.
   * @param [in] settings: Structure containing the settings for the ILQR algorithm.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  ILQR(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr, const constraint_base_t* systemConstraintsPtr,
       const cost_function_base_t* costFunctionPtr, const operating_trajectories_base_t* operatingTrajectoriesPtr,
       const ILQR_Settings& settings = ILQR_Settings(), const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  virtual ~ILQR() = default;

  void approximateOptimalControlProblem() override;

  void getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state, dynamic_vector_t& nu) const override;

  void calculateController() override;

  /**
   * Gets a reference to the Options structure.
   *
   * @return a reference to the Options structure.
   */
  ILQR_Settings& settings();

 protected:
  void setupOptimizer(size_t numPartitions) override;

  void approximateLQWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) override;

  void approximateUnconstrainedLQWorker(size_t workerIndex, size_t i, size_t k) override;

  void calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) override;

  void riccatiSolverTask() override;

  /**
   * Calculates the discrete-time LQ approximation from the continuous-time LQ approximation.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] i: Time partition index.
   * @param [in] k: Time index in the partition.
   */
  void discreteLQWorker(size_t workerIndex, size_t i, size_t k);

  /**
   * Solves a set of Riccati equations for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   */
  void riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
                              const eigen_scalar_t& sFinal);

  /****************
   *** Variables **
   ****************/
  ILQR_Settings settings_;

  // parallel Riccati solver
  std::mutex riccatiSolverDataMutex_;

  // Discrete-time components
  state_matrix_array2_t AmDtimeTrajectoryStock_;
  state_input_matrix_array2_t BmDtimeTrajectoryStock_;

  eigen_scalar_array2_t qDtimeTrajectoryStock_;
  state_vector_array2_t QvDtimeTrajectoryStock_;
  state_matrix_array2_t QmDtimeTrajectoryStock_;
  input_vector_array2_t RvDtimeTrajectoryStock_;
  input_matrix_array2_t RmDtimeTrajectoryStock_;
  input_state_matrix_array2_t PmDtimeTrajectoryStock_;
  input_matrix_array2_t RmInverseDtimeTrajectoryStock_;

  input_matrix_array2_t HmTrajectoryStock_;
  input_matrix_array2_t HmInverseTrajectoryStock_;
  input_state_matrix_array2_t GmTrajectoryStock_;
  input_vector_array2_t GvTrajectoryStock_;
};

}  // namespace ocs2

#include "implementation/ILQR.h"
