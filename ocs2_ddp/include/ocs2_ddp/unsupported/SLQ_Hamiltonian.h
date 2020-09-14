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

/*
 *  !! Copy of SLQ_BASE with the unfinished Hamiltonian implementation for the backward pass !!
 */

#pragma once

#include <ocs2_ddp/DDP_BASE.h>

#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/StateTriggeredEventHandler.h>
#include <ocs2_core/integration/SystemEventHandler.h>
#include <ocs2_core/misc/LTI_Equations.h>
#include <ocs2_core/misc/LinearAlgebra.h>

#include <ocs2_oc/rollout/StateTriggeredRollout.h>

#include <ocs2_ddp/SLQ_Settings.h>

#include <ocs2_ddp/riccati_equations/SequentialErrorEquationNormalized.h>
#include <ocs2_ddp/riccati_equations/SequentialRiccatiEquationsNormalized.h>

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T = NullLogicRules>
class SLQ_Hamiltonian : public DDP_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = DDP_BASE<STATE_DIM, INPUT_DIM>;

  using DIMENSIONS = typename BASE::DIMENSIONS;
  using controller_t = typename BASE::controller_t;
  using size_array_t = typename BASE::size_array_t;
  using size_array2_t = typename BASE::size_array2_t;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using scalar_array2_t = typename BASE::scalar_array2_t;
  using scalar_array3_t = typename BASE::scalar_array3_t;
  using eigen_scalar_t = typename BASE::eigen_scalar_t;
  using eigen_scalar_array_t = typename BASE::eigen_scalar_array_t;
  using eigen_scalar_array2_t = typename BASE::eigen_scalar_array2_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_vector_array_t = typename BASE::state_vector_array_t;
  using state_vector_array2_t = typename BASE::state_vector_array2_t;
  using state_vector_array3_t = typename BASE::state_vector_array3_t;
  using input_vector_t = typename BASE::input_vector_t;
  using input_vector_array_t = typename BASE::input_vector_array_t;
  using input_vector_array2_t = typename BASE::input_vector_array2_t;
  using input_vector_array3_t = typename BASE::input_vector_array3_t;
  using input_state_matrix_t = typename BASE::input_state_matrix_t;
  using input_state_matrix_array_t = typename BASE::input_state_matrix_array_t;
  using input_state_matrix_array2_t = typename BASE::input_state_matrix_array2_t;
  using input_state_matrix_array3_t = typename BASE::input_state_matrix_array3_t;
  using state_matrix_t = typename BASE::state_matrix_t;
  using state_matrix_array_t = typename BASE::state_matrix_array_t;
  using state_matrix_array2_t = typename BASE::state_matrix_array2_t;
  using state_matrix_array3_t = typename BASE::state_matrix_array3_t;
  using input_matrix_t = typename BASE::input_matrix_t;
  using input_matrix_array_t = typename BASE::input_matrix_array_t;
  using input_matrix_array2_t = typename BASE::input_matrix_array2_t;
  using input_matrix_array3_t = typename BASE::input_matrix_array3_t;
  using state_input_matrix_t = typename BASE::state_input_matrix_t;
  using state_input_matrix_array_t = typename BASE::state_input_matrix_array_t;
  using state_input_matrix_array2_t = typename BASE::state_input_matrix_array2_t;
  using state_input_matrix_array3_t = typename BASE::state_input_matrix_array3_t;
  using constraint1_vector_t = typename BASE::constraint1_vector_t;
  using constraint1_vector_array_t = typename BASE::constraint1_vector_array_t;
  using constraint1_vector_array2_t = typename BASE::constraint1_vector_array2_t;
  using constraint1_state_matrix_t = typename BASE::constraint1_state_matrix_t;
  using constraint1_state_matrix_array_t = typename BASE::constraint1_state_matrix_array_t;
  using constraint1_state_matrix_array2_t = typename BASE::constraint1_state_matrix_array2_t;
  using constraint1_input_matrix_t = typename BASE::constraint1_input_matrix_t;
  using constraint1_input_matrix_array_t = typename BASE::constraint1_input_matrix_array_t;
  using constraint1_input_matrix_array2_t = typename BASE::constraint1_input_matrix_array2_t;
  using input_constraint1_matrix_t = typename BASE::input_constraint1_matrix_t;
  using input_constraint1_matrix_array_t = typename BASE::input_constraint1_matrix_array_t;
  using input_constraint1_matrix_array2_t = typename BASE::input_constraint1_matrix_array2_t;
  using constraint2_vector_t = typename BASE::constraint2_vector_t;
  using constraint2_vector_array_t = typename BASE::constraint2_vector_array_t;
  using constraint2_vector_array2_t = typename BASE::constraint2_vector_array2_t;
  using constraint2_state_matrix_t = typename BASE::constraint2_state_matrix_t;
  using constraint2_state_matrix_array_t = typename BASE::constraint2_state_matrix_array_t;
  using constraint2_state_matrix_array2_t = typename BASE::constraint2_state_matrix_array2_t;
  using dynamic_vector_t = typename BASE::dynamic_vector_t;
  using dynamic_matrix_t = typename BASE::dynamic_matrix_t;
  using dynamic_vector_array_t = typename BASE::dynamic_vector_array_t;
  using dynamic_matrix_array2_t = typename BASE::dynamic_matrix_array2_t;

  using controller_ptr_array_t = typename BASE::controller_ptr_array_t;
  using linear_controller_t = typename BASE::linear_controller_t;
  using linear_controller_array_t = typename BASE::linear_controller_array_t;
  using event_handler_t = typename BASE::event_handler_t;
  using controlled_system_base_t = typename BASE::controlled_system_base_t;
  using derivatives_base_t = typename BASE::derivatives_base_t;
  using constraint_base_t = typename BASE::constraint_base_t;
  using cost_function_base_t = typename BASE::cost_function_base_t;
  using operating_trajectories_base_t = typename BASE::operating_trajectories_base_t;
  using penalty_base_t = typename BASE::penalty_base_t;
  using rollout_base_t = typename BASE::rollout_base_t;
  using time_triggered_rollout_t = typename BASE::time_triggered_rollout_t;
  using linear_quadratic_approximator_t = typename BASE::linear_quadratic_approximator_t;
  using operating_trajectorie_rollout_t = typename BASE::operating_trajectorie_rollout_t;
  using cost_desired_trajectories_t = typename BASE::cost_desired_trajectories_t;
  using logic_rules_machine_t = typename BASE::logic_rules_machine_t;
  using logic_rules_machine_ptr_t = typename BASE::logic_rules_machine_ptr_t;

  using hamiltonian_equation_t = LTI_Equations<2 * STATE_DIM, STATE_DIM, double>;
  using hamiltonian_increment_equation_t = LTI_Equations<STATE_DIM, 1, double>;

  /**
   * Default constructor.
   */
  SLQ_Hamiltonian() = default;

  /**
   * Constructor
   *
   * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and final costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
   * @param [in] settings: Structure containing the settings for the SLQ algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed-logic dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the final cost function defined in costFunctionPtr.
   */
  SLQ_Hamiltonian(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
                  const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                  const operating_trajectories_base_t* operatingTrajectoriesPtr, const SLQ_Settings& settings = SLQ_Settings(),
                  const LOGIC_RULES_T* logicRulesPtr = nullptr, const cost_function_base_t* heuristicsFunctionPtr = nullptr);

  /**
   * Default destructor.
   */
  virtual ~SLQ_Hamiltonian() = default;

  /**
   * Approximates the nonlinear problem as a linear-quadratic problem around the nominal
   * state and control trajectories. This method updates the following variables:
   * 	- linearized system model and constraints
   * 	- \f$ dxdt = A_m(t)x + B_m(t)u \f$.
   * 	- s.t. \f$ C_m(t)x + D_m(t)u + E_v(t) = 0 \f$ \\
   * 	-      \f$ F_m(t)x + H_v(t) = 0 \f$ .
   * 	- AmTrajectoryStock_: \f$ A_m\f$  matrix.
   * 	- BmTrajectoryStock_: \f$ B_m\f$  matrix.
   * 	- CmTrajectoryStock_: \f$ C_m\f$ matrix.
   * 	- DmTrajectoryStock_: \f$ D_m\f$ matrix.
   * 	- EvTrajectoryStock_: \f$ E_v\f$ vector.
   * 	- FmTrajectoryStock_: \f$ F_m\f$ vector.
   * 	- HvTrajectoryStock_: \f$ H_v\f$ vector.
   *
   * 	- quadratized intermediate cost function
   * 	- intermediate cost: \f$ q(t) + 0.5 xQ_m(t)x + x'Q_v(t) + u'P_m(t)x + 0.5u'R_m(t)u + u'R_v(t) \f$
   * 	- qTrajectoryStock_:  \f$ q\f$
   * 	- QvTrajectoryStock_: \f$ Q_v\f$ vector.
   * 	- QmTrajectoryStock_:\f$  Q_m\f$ matrix.
   * 	- PmTrajectoryStock_: \f$ P_m\f$ matrix.
   * 	- RvTrajectoryStock_: \f$ R_v\f$ vector.
   * 	- RmTrajectoryStock_: \f$ R_m\f$ matrix.
   * 	- RmInverseTrajectoryStock_: inverse of \f$ R_m\f$ matrix.
   *
   * 	- as well as the constrained coefficients of
   * 		- linearized system model
   * 		- quadratized intermediate cost function
   * 		- quadratized final cost
   *
   */
  void approximateOptimalControlProblem() override;

  /**
   * Calculates the controller. This method uses the following variables:
   * - constrained, linearized model
   * - constrained, quadratized cost
   *
   * The method modifies:
   * - nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
   * 				improves the constraints as well as the increment to the feed-forward control input.
   */
  void calculateController() override;

  /**
   * Gets a reference to the Options structure.
   *
   * @return a reference to the Options structure.
   */
  SLQ_Settings& settings();

 protected:
  /**
   * Sets up optimizer for different number of partitions.
   *
   * @param [in] numPartitions: number of partitions.
   */
  virtual void setupOptimizer(const size_t& numPartitions);

  /**
   * Computes the controller for a particular time partition
   *
   * @param partitionIndex: Time partition index
   */
  virtual void calculatePartitionController(const size_t& partitionIndex) = 0;

  /**
   * Calculates an LQ approximate of the optimal control problem at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index.
   * @param [in] timeIndex: Time index in the partition.
   */
  void approximateLQWorker(size_t workerIndex, const size_t& partitionIndex, const size_t& timeIndex) override;

  /**
   * Modify the unconstrained LQ coefficients to constrained ones.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] i: Time partition index.
   * @param [in] k: Time index in the partition.
   * @param [in] stateConstraintPenalty: State-only constraint penalty.
   */
  virtual void approximateConstrainedLQWorker(size_t workerIndex, const size_t& i, const size_t& k, const scalar_t& stateConstraintPenalty);

  /**
   * Calculates controller at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index
   * @param [in] timeIndex: Time index in the partition
   */
  void calculateControllerWorker(size_t workerIndex, const size_t& partitionIndex, const size_t& timeIndex) override;

  /**
   * Solves a set of Riccati equations for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   */
  void solveRiccatiEquationsWorker(size_t workerIndex, const size_t& partitionIndex, const state_matrix_t& SmFinal,
                                   const state_vector_t& SvFinal, const eigen_scalar_t& sFinal);

  /**
   * Solves a set of Riccati equations for the partition in the given index for nominal time trajectory stamp.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] nominalTimeTrajectory: The input array of the time trajectories.
   * @param [in] SmFinal: The final Sm for the current Riccati equation.
   * @param [in] SvFinal: The final Sv for the current Riccati equation.
   * @param [in] sFinal: The final s for the current Riccati equation.
   */
  void solveRiccatiEquationsForNominalTimeWorker(size_t workerIndex, const size_t& partitionIndex, const state_matrix_t& SmFinal,
                                                 const state_vector_t& SvFinal, const eigen_scalar_t& sFinal);

  /**
   * Type_1 constraints error correction compensation which solves a set of error Riccati equations for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SveFinal: The final Sve for the current Riccati equation.
   */
  void solveErrorRiccatiEquationWorker(size_t workerIndex, const size_t& partitionIndex, const state_vector_t& SveFinal);

  /**
   * Solves a set of Riccati equations and type_1 constraints error correction compensation for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   * @param [in] SveFinal: The final Sve for the current Riccati equation.
   */
  void solveSlqRiccatiEquationsWorker(size_t workerIndex, const size_t& partitionIndex, const state_matrix_t& SmFinal,
                                      const state_vector_t& SvFinal, const eigen_scalar_t& sFinal, const state_vector_t& SveFinal);

  /**
   * Full Backward Sweep method uses exponential method instead of ODE to solve Riccati equations.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SmFinal: The final Sm for the Riccati equation.
   * @param [in] SvFinal: The final Sv for the Riccati equation.
   * @param [in] SveFinal: The final Sve for the Riccati equation.
   * @param [in] sFinal: The final s for the Riccati equation.
   * @param [in] constraintStepSize: type-1 constraint step-size
   */
  void fullRiccatiBackwardSweepWorker(size_t workerIndex, const size_t& partitionIndex, const state_matrix_t& SmFinal,
                                      const state_vector_t& SvFinal, const state_vector_t& SveFinal, const eigen_scalar_t& sFinal,
                                      const scalar_t& constraintStepSize);

  template <int DIM1, int DIM2 = 1>
  Eigen::Matrix<scalar_t, DIM1, DIM2> solveLTI(const std::shared_ptr<IntegratorBase<DIM1 * DIM2>>& firstOrderOdeIntegrator,
                                               const Eigen::Matrix<scalar_t, DIM1, DIM2>& x0, const scalar_t& deltaTime);

  Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM> integrateHamiltonian(size_t workerIndex,
                                                                         const Eigen::Matrix<scalar_t, 2 * STATE_DIM, 2 * STATE_DIM>& Hm,
                                                                         const Eigen::Matrix<scalar_t, 2 * STATE_DIM, STATE_DIM>& x0,
                                                                         const scalar_t& deltaTime);

  Eigen::Matrix<scalar_t, STATE_DIM, 1> integrateIncrement(size_t workerIndex, const Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM>& Gm,
                                                           const Eigen::Matrix<scalar_t, STATE_DIM, 1>& Gv,
                                                           const Eigen::Matrix<scalar_t, STATE_DIM, 1>& x0, const scalar_t& deltaTime);

  /****************
   *** Variables **
   ****************/
  std::vector<std::shared_ptr<hamiltonian_equation_t>> hamiltonianEquationPtrStock_;
  std::vector<std::shared_ptr<IntegratorBase<hamiltonian_equation_t::LTI_DIM_>>> hamiltonianIntegratorPtrStock_;
  std::vector<std::shared_ptr<hamiltonian_increment_equation_t>> hamiltonianIncrementEquationPtrStock_;
  std::vector<std::shared_ptr<IntegratorBase<hamiltonian_increment_equation_t::LTI_DIM_>>> hamiltonianIncrementIntegratorPtrStock_;

  // function for Riccati error equation
  std::vector<EigenLinearInterpolation<state_matrix_t>> SmFuncs_;

  // Functions for solving Backward pass through Mobius scheme
  void LmFunc_(const size_t& partitionIndex, const size_t& timeIndex, input_state_matrix_t& Lm) {
    Lm = -RmInverseTrajectoryStock_[partitionIndex][timeIndex] *
         (BASE::PmTrajectoryStock_[partitionIndex][timeIndex] +
          BASE::BmTrajectoryStock_[partitionIndex][timeIndex].transpose() * BASE::SmTrajectoryStock_[partitionIndex][timeIndex]);
  };
  //
  void LmConstrainedFunc_(const size_t& partitionIndex, const size_t& timeIndex, const input_state_matrix_t& Lm,
                          input_state_matrix_t& LmConstrained) {
    LmConstrained = (input_matrix_t::Identity() - DmProjectedTrajectoryStock_[partitionIndex][timeIndex]) * Lm;
  };
  //
  void LvConstrainedFunc_(const size_t& partitionIndex, const size_t& timeIndex, input_vector_t& LvConstrained) {
    LvConstrained =
        -RmInverseTrajectoryStock_[partitionIndex][timeIndex] *
        (RvConstrainedTrajectoryStock_[partitionIndex][timeIndex] +
         BmConstrainedTrajectoryStock_[partitionIndex][timeIndex].transpose() * BASE::SvTrajectoryStock_[partitionIndex][timeIndex]);
  };
  //
  void LveConstrainedFunc_(const size_t& partitionIndex, const size_t& timeIndex, input_vector_t& LveConstrained) {
    LveConstrained = -RmInverseTrajectoryStock_[partitionIndex][timeIndex] *
                     BmConstrainedTrajectoryStock_[partitionIndex][timeIndex].transpose() *
                     BASE::SveTrajectoryStock_[partitionIndex][timeIndex];
  };
  //
  void ControllerFunc_(const size_t& partitionIndex, const size_t& timeIndex, const scalar_t& constraintStepSize,
                       const input_state_matrix_t& LmConstrained, const input_vector_t& LvConstrained,
                       const input_vector_t& LveConstrained) {
    // k
    BASE::nominalControllersStock_[partitionIndex].gainArray_[timeIndex] =
        LmConstrained - CmProjectedTrajectoryStock_[partitionIndex][timeIndex];
    // uff
    BASE::nominalControllersStock_[partitionIndex].biasArray_[timeIndex] =
        BASE::nominalInputTrajectoriesStock_[partitionIndex][timeIndex] -
        BASE::nominalControllersStock_[partitionIndex].gainArray_[timeIndex] *
            BASE::nominalStateTrajectoriesStock_[partitionIndex][timeIndex] +
        constraintStepSize * (LveConstrained - EvProjectedTrajectoryStock_[partitionIndex][timeIndex]);
    // deltaUff
    BASE::nominalControllersStock_[partitionIndex].deltaBiasArray_[timeIndex] = LvConstrained;
  };
};

}  // namespace ocs2

#include "implementation/SLQ_Hamiltonian.h"
