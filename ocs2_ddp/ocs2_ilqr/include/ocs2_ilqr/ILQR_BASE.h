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

#ifndef ILQR_BASE_OCS2_H_
#define ILQR_BASE_OCS2_H_

#include <ocs2_ddp_base/DDP_BASE.h>

#include <ocs2_ilqr/ILQR_Settings.h>


namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread ILQR.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class ILQR_BASE : public DDP_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef DDP_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	typedef typename BASE::DIMENSIONS                          DIMENSIONS;
	typedef typename BASE::controller_t                        controller_t;
	typedef typename BASE::size_array_t                        size_array_t;
	typedef typename BASE::size_array2_t                       size_array2_t;
	typedef typename BASE::scalar_t                            scalar_t;
	typedef typename BASE::scalar_array_t                      scalar_array_t;
	typedef typename BASE::scalar_array2_t                     scalar_array2_t;
	typedef typename BASE::scalar_array3_t                     scalar_array3_t;
	typedef typename BASE::eigen_scalar_t                      eigen_scalar_t;
	typedef typename BASE::eigen_scalar_array_t                eigen_scalar_array_t;
	typedef typename BASE::eigen_scalar_array2_t               eigen_scalar_array2_t;
	typedef typename BASE::state_vector_t                      state_vector_t;
	typedef typename BASE::state_vector_array_t                state_vector_array_t;
	typedef typename BASE::state_vector_array2_t               state_vector_array2_t;
	typedef typename BASE::state_vector_array3_t               state_vector_array3_t;
	typedef typename BASE::input_vector_t                      input_vector_t;
	typedef typename BASE::input_vector_array_t                input_vector_array_t;
	typedef typename BASE::input_vector_array2_t               input_vector_array2_t;
	typedef typename BASE::input_vector_array3_t               input_vector_array3_t;
	typedef typename BASE::input_state_matrix_t                input_state_matrix_t;
	typedef typename BASE::input_state_matrix_array_t          input_state_matrix_array_t;
	typedef typename BASE::input_state_matrix_array2_t         input_state_matrix_array2_t;
	typedef typename BASE::input_state_matrix_array3_t         input_state_matrix_array3_t;
	typedef typename BASE::state_matrix_t                      state_matrix_t;
	typedef typename BASE::state_matrix_array_t                state_matrix_array_t;
	typedef typename BASE::state_matrix_array2_t               state_matrix_array2_t;
	typedef typename BASE::state_matrix_array3_t               state_matrix_array3_t;
	typedef typename BASE::input_matrix_t                      input_matrix_t;
	typedef typename BASE::input_matrix_array_t                input_matrix_array_t;
	typedef typename BASE::input_matrix_array2_t               input_matrix_array2_t;
	typedef typename BASE::input_matrix_array3_t               input_matrix_array3_t;
	typedef typename BASE::state_input_matrix_t                state_input_matrix_t;
	typedef typename BASE::state_input_matrix_array_t          state_input_matrix_array_t;
	typedef typename BASE::state_input_matrix_array2_t         state_input_matrix_array2_t;
	typedef typename BASE::state_input_matrix_array3_t         state_input_matrix_array3_t;
	typedef typename BASE::constraint1_vector_t                constraint1_vector_t;
	typedef typename BASE::constraint1_vector_array_t          constraint1_vector_array_t;
	typedef typename BASE::constraint1_vector_array2_t         constraint1_vector_array2_t;
	typedef typename BASE::constraint1_state_matrix_t          constraint1_state_matrix_t;
	typedef typename BASE::constraint1_state_matrix_array_t    constraint1_state_matrix_array_t;
	typedef typename BASE::constraint1_state_matrix_array2_t   constraint1_state_matrix_array2_t;
	typedef typename BASE::constraint1_input_matrix_t          constraint1_input_matrix_t;
	typedef typename BASE::constraint1_input_matrix_array_t    constraint1_input_matrix_array_t;
	typedef typename BASE::constraint1_input_matrix_array2_t   constraint1_input_matrix_array2_t;
	typedef typename BASE::control_constraint1_matrix_t        control_constraint1_matrix_t;
	typedef typename BASE::control_constraint1_matrix_array_t  control_constraint1_matrix_array_t;
	typedef typename BASE::control_constraint1_matrix_array2_t control_constraint1_matrix_array2_t;
	typedef typename BASE::constraint2_vector_t                constraint2_vector_t;
	typedef typename BASE::constraint2_vector_array_t          constraint2_vector_array_t;
	typedef typename BASE::constraint2_vector_array2_t         constraint2_vector_array2_t;
	typedef typename BASE::constraint2_state_matrix_t          constraint2_state_matrix_t;
	typedef typename BASE::constraint2_state_matrix_array_t    constraint2_state_matrix_array_t;
	typedef typename BASE::constraint2_state_matrix_array2_t   constraint2_state_matrix_array2_t;
	typedef typename BASE::dynamic_vector_t                    dynamic_vector_t;
	typedef typename BASE::dynamic_vector_array_t              dynamic_vector_array_t;

	typedef typename BASE::controller_ptr_array_t              controller_ptr_array_t;
	typedef typename BASE::linear_controller_t                 linear_controller_t;
	typedef typename BASE::linear_controller_array_t           linear_controller_array_t;
	typedef typename BASE::event_handler_t                     event_handler_t;
	typedef typename BASE::controlled_system_base_t            controlled_system_base_t;
	typedef typename BASE::derivatives_base_t                  derivatives_base_t;
	typedef typename BASE::constraint_base_t                   constraint_base_t;
	typedef typename BASE::cost_function_base_t                cost_function_base_t;
	typedef typename BASE::operating_trajectories_base_t       operating_trajectories_base_t;
	typedef typename BASE::penalty_base_t                      penalty_base_t;
	typedef typename BASE::rollout_base_t                      rollout_base_t;
	typedef typename BASE::time_triggered_rollout_t            time_triggered_rollout_t;
	typedef typename BASE::linear_quadratic_approximator_t     linear_quadratic_approximator_t;
	typedef typename BASE::operating_trajectorie_rollout_t     operating_trajectorie_rollout_t;
	typedef typename BASE::cost_desired_trajectories_t         cost_desired_trajectories_t;
	typedef typename BASE::logic_rules_machine_t               logic_rules_machine_t;
	typedef typename BASE::logic_rules_machine_ptr_t           logic_rules_machine_ptr_t;

//	/**
//	 * class for collecting ILQR data
//	 */
//	template <size_t OTHER_STATE_DIM, size_t OTHER_INPUT_DIM, class OTHER_LOGIC_RULES_T>
//	friend class SLQ_DataCollector;

	/**
	 * Default constructor.
	 */
	ILQR_BASE() = default;

	/**
	 * Constructor
	 *
	 * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
	 * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
	 * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
	 * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
	 * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of ILQR.
	 * @param [in] settings: Structure containing the settings for the ILQR algorithm.
	 * @param [in] logicRulesPtr: The logic rules used for implementing mixed-logic dynamical systems.
	 * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
	 * defined, we will use the terminal cost function defined in costFunctionPtr.
	 */
	ILQR_BASE (const controlled_system_base_t* systemDynamicsPtr,
			  const derivatives_base_t* systemDerivativesPtr,
			  const constraint_base_t* systemConstraintsPtr,
			  const cost_function_base_t* costFunctionPtr,
			  const operating_trajectories_base_t* operatingTrajectoriesPtr,
			  const ILQR_Settings& settings = ILQR_Settings(),
			  const LOGIC_RULES_T* logicRulesPtr = nullptr,
			  const cost_function_base_t* heuristicsFunctionPtr = nullptr);

	/**
	 * Default destructor.
	 */
	virtual ~ILQR_BASE() = default;

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
	 */
	virtual void approximateOptimalControlProblem() override;

	/**
	 * Calculates the controller. This method uses the following variables:
	 * - constrained, linearized model
	 * - constrained, quadratized cost
	 *
	 * The method modifies:
	 * - nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
	 * 								improves the constraints as well as the increment to the feed-forward control input.
	 */
	void calculateController();

	/**
	 * Line search on the feedforward parts of the controller. It uses the following approach for line search:
	 * The constraint TYPE-1 correction term is directly added through a user defined stepSize (defined in settings_.constraintStepSize_).
	 * But the cost minimization term is optimized through a line-search strategy defined in ILQR settings.
	 *
	 * @param [in] computeISEs: Whether lineSearch needs to calculate ISEs indices for type_1 and type-2 constraints.
	 */
	virtual void lineSearch(bool computeISEs) = 0;

	/**
	 * Gets a reference to the Options structure.
	 *
	 * @return a reference to the Options structure.
	 */
	ILQR_Settings& settings();


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
	virtual void approximateLQWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const size_t& timeIndex) override;

	/**
	 * Calculates an LQ approximate of the unconstrained optimal control problem at a given partition and a node.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] i: Time partition index.
	 * @param [in] k: Time index in the partition.
	 */
	virtual void approximateUnconstrainedLQWorker(
			size_t workerIndex,
			const size_t& i,
			const size_t& k) override;

	/**
	 * Calculates the discrete-time LQ approximation from the continuous-time LQ approximation.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] i: Time partition index.
	 * @param [in] k: Time index in the partition.
	 */
	void discreteLQWorker(
			size_t workerIndex,
			const size_t& i,
			const size_t& k);

	/**
	 * Calculates controller at a given partition and a node.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: Time partition index
	 * @param [in] timeIndex: Time index in the partition
	 */
	virtual void calculateControllerWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const size_t& timeIndex) override;

	/**
	 * Solves a set of Riccati equations for the partition in the given index.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
	 * @param [in] SmFinal: The final Sm for Riccati equation.
	 * @param [in] SvFinal: The final Sv for Riccati equation.
	 * @param [in] sFinal: The final s for Riccati equation.
	 */
	void solveRiccatiEquationsWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const state_matrix_t& SmFinal,
			const state_vector_t& SvFinal,
			const eigen_scalar_t& sFinal);

	/**
	 * Type_1 constraints error correction compensation which solves a set of error Riccati equations for the partition in the given index.
	 *
	 * @param [in] workerIndex: Working agent index.
	 * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
	 * @param [in] SveFinal: The final Sve for the current Riccati equation.
	 */
	void solveErrorRiccatiEquationWorker(
			size_t workerIndex,
			const size_t& partitionIndex,
			const state_vector_t& SveFinal);

	/****************
	 *** Variables **
	 ****************/
	ILQR_Settings settings_;

	// Discrete-time components
	state_matrix_array2_t       AmDtimeTrajectoryStock_;
	state_input_matrix_array2_t BmDtimeTrajectoryStock_;

	eigen_scalar_array2_t       qDtimeTrajectoryStock_;
	state_vector_array2_t       QvDtimeTrajectoryStock_;
	state_matrix_array2_t       QmDtimeTrajectoryStock_;
	input_vector_array2_t       RvDtimeTrajectoryStock_;
	input_matrix_array2_t       RmDtimeTrajectoryStock_;
	input_state_matrix_array2_t PmDtimeTrajectoryStock_;
	input_matrix_array2_t       RmInverseDtimeTrajectoryStock_;

	input_matrix_array2_t       HmTrajectoryStock_;
	input_matrix_array2_t       HmInverseTrajectoryStock_;
	input_state_matrix_array2_t GmTrajectoryStock_;
	input_vector_array2_t       GvTrajectoryStock_;
};

} // namespace ocs2

#include "implementation/ILQR_BASE.h"

#endif /* ILQR_BASE_OCS2_H_ */
