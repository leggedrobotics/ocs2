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

#ifndef SOLVER_BASE_OCS2_H_
#define SOLVER_BASE_OCS2_H_

#include <array>
#include <mutex>
#include <algorithm>
#include <numeric>
#include <cstddef>
#include <Eigen/StdVector>
#include <vector>
#include <type_traits>
#include <chrono>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/logic/rules/LogicRulesBase.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>
#include <ocs2_core/logic/machine/LogicRulesMachine.h>
#include <ocs2_core/misc/FindActiveIntervalIndex.h>
#include <ocs2_core/misc/LinearAlgebra.h>

namespace ocs2 {

/**
 * This class is an interface class for the single-thread and multi-thread SLQ.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class Solver_BASE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	static_assert(std::is_base_of<LogicRulesBase, LOGIC_RULES_T>::value,
			"LOGIC_RULES_T must inherit from LogicRulesBase");

	typedef std::shared_ptr<Solver_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::size_array_t                        size_array_t;
	typedef typename DIMENSIONS::size_array2_t                       size_array2_t;
	typedef typename DIMENSIONS::scalar_t                            scalar_t;
	typedef typename DIMENSIONS::scalar_array_t                      scalar_array_t;
	typedef typename DIMENSIONS::scalar_array2_t                     scalar_array2_t;
	typedef typename DIMENSIONS::scalar_array3_t                     scalar_array3_t;
	typedef typename DIMENSIONS::eigen_scalar_t                      eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t                eigen_scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_array2_t               eigen_scalar_array2_t;
	typedef typename DIMENSIONS::state_vector_t                      state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t                state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t               state_vector_array2_t;
	typedef typename DIMENSIONS::state_vector_array3_t               state_vector_array3_t;
	typedef typename DIMENSIONS::input_vector_t                      input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t                input_vector_array_t;
	typedef typename DIMENSIONS::input_vector_array2_t               input_vector_array2_t;
	typedef typename DIMENSIONS::input_vector_array3_t               input_vector_array3_t;
	typedef typename DIMENSIONS::input_state_matrix_t                input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t          input_state_matrix_array_t;
	typedef typename DIMENSIONS::input_state_matrix_array2_t         input_state_matrix_array2_t;
	typedef typename DIMENSIONS::input_state_matrix_array3_t         input_state_matrix_array3_t;
	typedef typename DIMENSIONS::state_matrix_t                      state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t                state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_array2_t               state_matrix_array2_t;
	typedef typename DIMENSIONS::state_matrix_array3_t               state_matrix_array3_t;
	typedef typename DIMENSIONS::input_matrix_t                      input_matrix_t;
	typedef typename DIMENSIONS::input_matrix_array_t                input_matrix_array_t;
	typedef typename DIMENSIONS::input_matrix_array2_t               input_matrix_array2_t;
	typedef typename DIMENSIONS::input_matrix_array3_t               input_matrix_array3_t;
	typedef typename DIMENSIONS::state_input_matrix_t                state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t          state_input_matrix_array_t;
	typedef typename DIMENSIONS::state_input_matrix_array2_t         state_input_matrix_array2_t;
	typedef typename DIMENSIONS::state_input_matrix_array3_t         state_input_matrix_array3_t;
	typedef typename DIMENSIONS::constraint1_vector_t                constraint1_vector_t;
	typedef typename DIMENSIONS::constraint1_vector_array_t          constraint1_vector_array_t;
	typedef typename DIMENSIONS::constraint1_vector_array2_t         constraint1_vector_array2_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t          constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array_t    constraint1_state_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array2_t   constraint1_state_matrix_array2_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_t          constraint1_input_matrix_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_array_t    constraint1_input_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_input_matrix_array2_t   constraint1_input_matrix_array2_t;
	typedef typename DIMENSIONS::input_constraint1_matrix_t          input_constraint1_matrix_t;
	typedef typename DIMENSIONS::input_constraint1_matrix_array_t    input_constraint1_matrix_array_t;
	typedef typename DIMENSIONS::input_constraint1_matrix_array2_t   input_constraint1_matrix_array2_t;
	typedef typename DIMENSIONS::constraint2_vector_t                constraint2_vector_t;
	typedef typename DIMENSIONS::constraint2_vector_array_t          constraint2_vector_array_t;
	typedef typename DIMENSIONS::constraint2_vector_array2_t         constraint2_vector_array2_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_t          constraint2_state_matrix_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_array_t    constraint2_state_matrix_array_t;
	typedef typename DIMENSIONS::constraint2_state_matrix_array2_t   constraint2_state_matrix_array2_t;
	typedef typename DIMENSIONS::dynamic_vector_t                    dynamic_vector_t;
	typedef typename DIMENSIONS::dynamic_matrix_t                    dynamic_matrix_t;
	typedef typename DIMENSIONS::dynamic_vector_array_t              dynamic_vector_array_t;
  	typedef typename DIMENSIONS::dynamic_matrix_array2_t             dynamic_matrix_array2_t;

	typedef CostDesiredTrajectories<scalar_t> cost_desired_trajectories_t;

	typedef LogicRulesMachine<LOGIC_RULES_T>     logic_rules_machine_t;
	typedef typename logic_rules_machine_t::Ptr	 logic_rules_machine_ptr_t;

	typedef ControllerBase<STATE_DIM, INPUT_DIM> controller_t;
	typedef std::vector<controller_t*>       controller_ptr_array_t;

	/**
	 * Default constructor.
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
	virtual void run(
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes) = 0;

	/**
	 * The main routine of solver which runs the optimizer for a given initial state, initial time, final time, and
	 * initial controller.
	 *
	 * @param [in] initTime: The initial time.
	 * @param [in] initState: The initial state.
	 * @param [in] finalTime: The final time.
	 * @param [in] partitioningTimes: The time partitioning.
	 * @param [in] controllersPtrStock: controllersPtrStock: Array of pointers to the initial control policies. If you want to use the control policy
	 * which was designed by the previous call of the "run" routine, you should pass an empty array.
	 * In the this case, two scenarios are possible: either the internal controller is already set (such as the MPC case
	 * where the warm starting option is set true) or the internal controller is empty in which instead of performing
	 * a rollout the operating trajectories will be used.
	 */
	virtual void run(
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes,
			const controller_ptr_array_t& controllersPtrStock) = 0;

	/**
	 * MPC_BASE activates this if the final time of the MPC will increase by the length of a time partition instead
	 * of commonly used scheme where the final time is gradually increased.
	 *
	 * @param [in] flag: If set true, the final time of the MPC will increase by the length of a time partition.
	 */
	virtual void blockwiseMovingHorizon(bool flag) = 0;

	/**
	 * Gets the cost function and ISEs of the type-1 and type-2 constraints at the initial time.
	 *
	 * @param [out] costFunction: cost function value
	 * @param [out] constraint1ISE: type-1 constraint ISE.
	 * @param [out] constraint1ISE: type-2 constraint ISE.
	 */
	virtual void getPerformanceIndeces(
			scalar_t& costFunction,
			scalar_t& constraint1ISE,
			scalar_t& constraint2ISE) const = 0;

	/**
	 * Gets number of iterations.
	 *
	 * @return Number of iterations.
	 */
	virtual size_t getNumIterations() const = 0;

	/**
	 * Gets iterations Log of SLQ.
	 *
	 * @param [out] iterationCost: Each iteration's cost.
	 * @param [out] iterationISE1: Each iteration's type-1 constraints ISE.
	 * @param [out] iterationISE2: Each iteration's type-2 constraints ISE.
	 */
	virtual void getIterationsLog(
			eigen_scalar_array_t& iterationCost,
			eigen_scalar_array_t& iterationISE1,
			eigen_scalar_array_t& iterationISE2) const = 0;

	/**
	 * Gets Iterations Log of SLQ
	 *
	 * @param [out] iterationCostPtr: A pointer to each iteration's cost.
	 * @param [out] iterationISE1Ptr: A pointer to each iteration's type-1 constraints ISE.
	 * @param [out] iterationISE2Ptr: A pointer to each iteration's type-2 constraints ISE.
	 */
	virtual void getIterationsLogPtr(
			const eigen_scalar_array_t*& iterationCostPtr,
			const eigen_scalar_array_t*& iterationISE1Ptr,
			const eigen_scalar_array_t*& iterationISE2Ptr) const = 0;

	/**
	 * Gets final time of optimization
	 *
	 * @return finalTime
	 */
	virtual const scalar_t& getFinalTime() const = 0;

	/**
	 * Returns the final time of optimization.
	 *
	 * @return finalTime
	 */
	virtual const scalar_array_t& getPartitioningTimes() const = 0;

	/**
	 * Returns a pointer to the LogicRulesMachine
	 *
	 * @return a pointer to LogicRulesMachine
	 */
	virtual logic_rules_machine_t* getLogicRulesMachinePtr() {

		return nullptr;
	}

	/**
	 * Returns a pointer to the LogicRulesMachine
	 *
	 * @return a pointer to LogicRulesMachine
	 */
	virtual const logic_rules_machine_t* getLogicRulesMachinePtr() const {

		return nullptr;
	}

	/**
	 * Returns a constant pointer to the logic rules.
	 *
	 * @return a constant pointer to the logic rules.
	 */
	virtual const LOGIC_RULES_T* getLogicRulesPtr() const {

		return nullptr;
	}

	/**
	 * Returns a pointer to the logic rules.
	 *
	 * @return a pointer to the logic rules.
	 */
	virtual LOGIC_RULES_T* getLogicRulesPtr() {

		return nullptr;
	}

	/**
	 * Sets logic rules.
	 *
	 * @param logicRules: This class will be passed to all of the dynamics and derivatives classes through initializeModel() routine.
	 */
	virtual void setLogicRules(const LOGIC_RULES_T& logicRules) {}

	/**
	 * Gets the cost function desired trajectories.
	 *
	 * @param [out] costDesiredTrajectories: A pointer to the cost function desired trajectories
	 */
	virtual void getCostDesiredTrajectoriesPtr(
			const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr) const = 0;

	/**
	 * Sets the cost function desired trajectories.
	 *
	 * @param [in] costDesiredTrajectories: The cost function desired trajectories
	 */
	virtual void setCostDesiredTrajectories(
			const cost_desired_trajectories_t& costDesiredTrajectories) = 0;

	/**
	 * Sets the cost function desired trajectories.
	 *
	 * @param [in] desiredTimeTrajectory: The desired time trajectory for cost.
	 * @param [in] desiredStateTrajectory: The desired state trajectory for cost.
	 * @param [in] desiredInputTrajectory: The desired input trajectory for cost.
	 */
	virtual void setCostDesiredTrajectories(
			const scalar_array_t& desiredTimeTrajectory,
			const dynamic_vector_array_t& desiredStateTrajectory,
			const dynamic_vector_array_t& desiredInputTrajectory) = 0;

	/**
	 * Swaps the cost function desired trajectories.
	 *
	 * @param [in] costDesiredTrajectories: The cost function desired trajectories
	 */
	virtual void swapCostDesiredTrajectories(
			cost_desired_trajectories_t& costDesiredTrajectories) = 0;

	/**
	 * Swaps the cost function desired trajectories.
	 *
	 * @param [in] desiredTimeTrajectory: The desired time trajectory for cost.
	 * @param [in] desiredStateTrajectory: The desired state trajectory for cost.
	 * @param [in] desiredInputTrajectory: The desired input trajectory for cost.
	 */
	virtual void swapCostDesiredTrajectories(
			scalar_array_t& desiredTimeTrajectory,
			dynamic_vector_array_t& desiredStateTrajectory,
			dynamic_vector_array_t& desiredInputTrajectory) = 0;

	/**
	 * Whether the cost function desired trajectories is updated.
	 *
	 * @return true if it is updated.
	 */
	virtual bool costDesiredTrajectoriesUpdated() const = 0;

	/**
	 * Returns an array of pointer to the optimal control policies.
	 *
	 * @return An array of pointers to the optimized control policies.
	 */
	virtual const controller_ptr_array_t& getController() const = 0;

	/**
	 * Gets an array of pointer to the optimal control policies.
	 *
	 * @param [out] controllersStockPtr: An array of pointers to the optimized control policies.
	 */
	virtual void getControllerPtr(const controller_ptr_array_t*& controllersPtrStock) const = 0;

	/**
	 * Returns the nominal time trajectories.
	 *
	 * @return nominalTimeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 */
	virtual const std::vector<scalar_array_t>& getNominalTimeTrajectories() const = 0;

	/**
	 * Returns the nominal state trajectories.
	 *
	 * @return nominalStateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 */
	virtual const state_vector_array2_t& getNominalStateTrajectories() const = 0;

	/**
	 * Returns the nominal input trajectories.
	 *
	 * @return nominalInputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
	 */
	virtual const input_vector_array2_t& getNominalInputTrajectories() const = 0;

	/**
	 * Gets a pointer to the nominal time, state, and input trajectories.
	 *
	 * @param [out] nominalTimeTrajectoriesStockPtr: A pointer to an array of trajectories containing the output time trajectory stamp.
	 * @param [out] nominalStateTrajectoriesStockPtr: A pointer to an array of trajectories containing the output state trajectory.
	 * @param [out] nominalInputTrajectoriesStockPtr: A pointer to an array of trajectories containing the output control input trajectory.
	 */
	virtual void getNominalTrajectoriesPtr(
			const std::vector<scalar_array_t>*& nominalTimeTrajectoriesStockPtr,
			const state_vector_array2_t*& nominalStateTrajectoriesStockPtr,
			const input_vector_array2_t*& nominalInputTrajectoriesStockPtr) const = 0;

	/**
	 * Swaps the the outputs with the nominal trajectories.
	 * Care should be take since this method modifies the internal variable.
	 *
	 * @param [out] nominalTimeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
	 * @param [out] nominalStateTrajectoriesStock: Array of trajectories containing the output state trajectory.
	 * @param [out] nominalInputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
	 */
	virtual void swapNominalTrajectories (
			std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
			state_vector_array2_t& nominalStateTrajectoriesStock,
			input_vector_array2_t& nominalInputTrajectoriesStock) = 0;

	/**
	 * Rewinds optimizer internal variables.
	 *
	 * @param [in] firstIndex: The index which we want to rewind to.
	 */
	virtual void rewindOptimizer(const size_t& firstIndex) = 0;

	/**
	 * Get rewind counter.
	 *
	 * @return Number of partition rewinds since construction of the class.
	 */
	virtual const unsigned long long int& getRewindCounter() const = 0;

	/**
	 * Finds the interval of partitioningTimes to which the input time belongs to it.
	 * time is in interval i if: partitioningTimes[i] < t <= partitioningTimes[i+1]
	 * Exception: if time=partitioningTimes[0] then time is interval 0
	 *
	 * @param [in] partitioningTimes: a sorted time sequence.
	 * @param [in] time: Enquiry time.
	 * @param [in] ceilingFunction: Use the ceiling function settings ().
	 * @return Active subsystem index.
	 */
	static size_t findActivePartitionIndex(
			const scalar_array_t& partitioningTimes,
			const scalar_t& time,
			bool ceilingFunction = true);

	/**
	 * Prints to output.
	 *
	 * @param [in] input text.
	 */
	void printString(const std::string& text);


private:
	std::mutex outputDisplayGuardMutex_;

};

} // namespace ocs2

#include "implementation/Solver_BASE.h"

#endif /* SOLVER_BASE_OCS2_H_ */
