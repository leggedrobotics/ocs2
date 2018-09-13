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

#ifndef MPC_SLQ_OCS2_H_
#define MPC_SLQ_OCS2_H_

#include <ocs2_core/Dimensions.h>
#include <ocs2_slq/SLQ_BASE.h>
#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>

#include "ocs2_mpc/MPC_BASE.h"

namespace ocs2{

/**
 * This an MPC implementation with SLQ optimal control solver.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class MPC_SLQ : public MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t		controller_t;
	typedef typename DIMENSIONS::controller_array_t	controller_array_t;
	typedef typename DIMENSIONS::scalar_t		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t	scalar_array_t;
	typedef typename DIMENSIONS::size_array_t	size_array_t;
	typedef typename DIMENSIONS::state_vector_t 		state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t	state_vector_array_t;
	typedef typename DIMENSIONS::state_vector_array2_t 	state_vector_array2_t;
	typedef typename DIMENSIONS::input_vector_t 		 input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t  input_vector_array_t;
	typedef typename DIMENSIONS::input_vector_array2_t input_vector_array2_t;
	typedef typename DIMENSIONS::input_state_matrix_t 	   input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t  input_state_matrix_array_t;
	typedef typename DIMENSIONS::input_state_matrix_array2_t input_state_matrix_array2_t;
	typedef typename DIMENSIONS::dynamic_vector_t       dynamic_vector_t;
	typedef typename DIMENSIONS::dynamic_vector_array_t dynamic_vector_array_t;

	typedef CostDesiredTrajectories<scalar_t> cost_desired_trajectories_t;

	typedef ocs2::SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>	slq_base_t;
	typedef ocs2::SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>		slq_t;
	typedef ocs2::SLQ_MP<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>	slq_mp_t;

	typedef ModeSequenceTemplate<scalar_t>					mode_sequence_template_t;
	typedef typename slq_base_t::logic_rules_machine_t		logic_rules_machine_t;
	typedef typename slq_base_t::controlled_system_base_t	controlled_system_base_t;
	typedef typename slq_base_t::event_handler_t			event_handler_t;
	typedef typename slq_base_t::derivatives_base_t			derivatives_base_t;
	typedef typename slq_base_t::constraint_base_t			constraint_base_t;
	typedef typename slq_base_t::cost_function_base_t		cost_function_base_t;
	typedef typename slq_base_t::operating_trajectories_base_t operating_trajectories_base_t;

	/**
	 * Default constructor.
	 */
	MPC_SLQ();

	/**
	 * Constructor
	 *
	 * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
	 * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
	 * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
	 * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
	 * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization of SLQ.
	 * @param [in] partitioningTimes: This will be used as the initial time partitioning. As the MPC progresses the internal
	 * partitioningTimes will be shifted in time automatically.
	 * @param [in] slqSettings: Structure containing the settings for the SLQ algorithm.
	 * @param [in] mpcSettings: Structure containing the settings for the MPC algorithm.
	 * @param [in] logicRulesPtr: The logic rules used for implementing mixed logical dynamical systems.
	 * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
	 * defined, we will use the terminal cost function defined in costFunctionPtr.
	 */
	MPC_SLQ(const controlled_system_base_t* systemDynamicsPtr,
			const derivatives_base_t* systemDerivativesPtr,
			const constraint_base_t* systemConstraintsPtr,
			const cost_function_base_t* costFunctionPtr,
			const operating_trajectories_base_t* operatingTrajectoriesPtr,
			const scalar_array_t& partitioningTimes,
			const SLQ_Settings& slqSettings = SLQ_Settings(),
			const MPC_Settings& mpcSettings = MPC_Settings(),
			const LOGIC_RULES_T* logicRulesPtr = nullptr,
			const mode_sequence_template_t* modeSequenceTemplatePtr = nullptr,
			const cost_function_base_t* heuristicsFunctionPtr = nullptr);

	/**
	 * destructor.
	 */
	virtual ~MPC_SLQ() = default;

	/**
	 * Resets the class to its state after construction.
	 */
	virtual void reset() override;

	/**
	 * Returns the initial time for which the optimizer is called.
	 *
	 * @return Initial time
	 */
	virtual scalar_t getStartTime() const override;

	/**
	 * Returns the final time for which the optimizer is called.
	 *
	 * @return Final time
	 */
	virtual scalar_t getFinalTime() const override;

	/**
	 * Returns the time horizon for which the optimizer is called.
	 *
	 * @return Time horizon
	 */
	virtual scalar_t getTimeHorizon() const override;

	/**
	 * Gets the SLQ settings structure.
	 *
	 * @return SLQ settings structure
	 */
	virtual SLQ_Settings& slqSettings();

	/**
	 * Gets partitioning time.
	 *
	 * @param [out] Partitioning times
	 */
	virtual void getPartitioningTimes(scalar_array_t& partitioningTimes) const;

	/**
	 * Gets an array of times indicating event times.
	 *
	 * @return eventTimes: Array of the event times.
	 */
	virtual const scalar_array_t& getEventTimes() const override;

	/**
	 * set logic rules.
	 *
	 * @param logicRules: This class will be passed to all of the dynamics and derivatives classes through initializeModel() routine.
	 */
	virtual void setLogicRules(const LOGIC_RULES_T& logicRules) override;

	/**
	 * get logic rules.
	 *
	 * @return logicRules.
	 */
	virtual const LOGIC_RULES_T& getLogicRules() const override;

	/**
	 * Gets the cost function desired trajectories.
	 *
	 * @param [out] costDesiredTrajectories: A pointer to the cost function desired trajectories
	 */
	virtual void getCostDesiredTrajectoriesPtr(
			const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr) const override;

	/**
	 * Sets the cost function desired trajectories.
	 *
	 * @param [in] costDesiredTrajectories: The cost function desired trajectories
	 */
	virtual void setCostDesiredTrajectories(
			const cost_desired_trajectories_t& costDesiredTrajectories) override;

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
			const dynamic_vector_array_t& desiredInputTrajectory) override;

	/**
	 * Swaps the cost function desired trajectories.
	 *
	 * @param [in] costDesiredTrajectories: The cost function desired trajectories
	 */
	virtual void swapCostDesiredTrajectories(
			cost_desired_trajectories_t& costDesiredTrajectories) override;

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
			dynamic_vector_array_t& desiredInputTrajectory) override;

	/**
	 * Solves the optimal control problem for the given state and time period ([initTime,finalTime]).
	 *
	 * @param [out] initTime: Initial time. This value can be adjusted by the optimizer.
	 * @param [in] initState: Initial state.
	 * @param [out] finalTime: Final time. This value can be adjusted by the optimizer.
	 * @param [out] timeTrajectoriesStock_out: A pointer to the optimized time trajectories.
	 * @param [out] stateTrajectoriesStock_out: A pointer to the optimized state trajectories.
	 * @param [out] inputTrajectoriesStock_out: A pointer to the optimized input trajectories.
	 * @param [out] controllerStock_out: A pointer to the optimized control policy.
	 */
	virtual void calculateController(
			scalar_t& initTime,
			const state_vector_t& initState,
			scalar_t& finalTime,
			const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
			const state_vector_array2_t*& stateTrajectoriesStockPtr,
			const input_vector_array2_t*& inputTrajectoriesStockPtr,
			const controller_array_t*& controllerStockPtr) override;

protected:
	/**
	 * Rewinds the SLQ-MPC.
	 */
	virtual void rewind();

	/**
	 * Adjustments time horizon.
	 *
	 * @param [in] partitioningTimes: Partitioning times after rewind.
	 * @param [out] initTime: Adjustments initial time.
	 * @param [out] finalTime: Adjustments final time.
	 * @param [out] initActivePartitionIndex: Index of the initial active partition.
	 * @param [out] finalActivePartitionIndex: Index of the final active partition.
	 */
	virtual void adjustmentTimeHorizon(
			const scalar_array_t& partitioningTimes,
			scalar_t& initTime,
			scalar_t& finalTime,
			size_t& initActivePartitionIndex,
			size_t& finalActivePartitionIndex) const;

	/***********
	 * Variabes
	 ***********/
	typename slq_base_t::Ptr slqPtr_;

	size_t initnumPartitions_;
	scalar_array_t initPartitioningTimes_;
	size_t numPartitions_;
	scalar_array_t partitioningTimes_;

	controller_array_t nullControllersStock_;

	size_t initActivePartitionIndex_;
	size_t finalActivePartitionIndex_;

	scalar_t lastControlDesignTime_;

	std::vector<scalar_array_t> optimizedTimeTrajectoriesStock_;
	state_vector_array2_t 		optimizedStateTrajectoriesStock_;
	input_vector_array2_t 		optimizedInputTrajectoriesStock_;

};

} // namespace ocs2

#include "implementation/MPC_SLQ.h"

#endif /* MPC_SLQ_OCS2_H_ */
