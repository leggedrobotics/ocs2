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

#ifndef SLQ_DATACOLLECTOR_OCS2_H_
#define SLQ_DATACOLLECTOR_OCS2_H_

#include <vector>
#include <Eigen/Dense>

#include <ocs2_slq/SLQ_BASE.h>

namespace ocs2 {

/**
 * Collects the required data from SLQ instance. It uses swap method wherever it is possible.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
  */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SLQ_DataCollector
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef SLQ_BASE<STATE_DIM, INPUT_DIM> slq_t;

	typedef std::shared_ptr<SLQ_DataCollector<STATE_DIM, INPUT_DIM>> Ptr;

	using linear_controller_array_t = typename slq_t::linear_controller_array_t;
	using size_array_t = typename slq_t::size_array_t;
	using scalar_t = typename slq_t::scalar_t;
	using scalar_array_t = typename slq_t::scalar_array_t;
	using scalar_array3_t = typename slq_t::scalar_array3_t;
	using eigen_scalar_t = typename slq_t::eigen_scalar_t;
	using eigen_scalar_array_t = typename slq_t::eigen_scalar_array_t;
	using eigen_scalar_array2_t = typename slq_t::eigen_scalar_array2_t;
	using state_vector_t = typename slq_t::state_vector_t;
	using state_vector_array_t = typename slq_t::state_vector_array_t;
	using state_vector_array2_t = typename slq_t::state_vector_array2_t;
	using state_vector_array3_t = typename slq_t::state_vector_array3_t;
	using input_vector_array_t = typename slq_t::input_vector_array_t;
	using input_vector_array2_t = typename slq_t::input_vector_array2_t;
	using input_vector_array3_t = typename slq_t::input_vector_array3_t;
	using input_state_matrix_array_t = typename slq_t::input_state_matrix_array_t;
	using input_state_matrix_array2_t = typename slq_t::input_state_matrix_array2_t;
	using input_state_matrix_array3_t = typename slq_t::input_state_matrix_array3_t;
	using state_matrix_array_t = typename slq_t::state_matrix_array_t;
	using state_matrix_t = typename slq_t::state_matrix_t;
	using state_matrix_array2_t = typename slq_t::state_matrix_array2_t;
	using state_matrix_array3_t = typename slq_t::state_matrix_array3_t;
	using input_matrix_array_t = typename slq_t::input_matrix_array_t;
	using input_matrix_array2_t = typename slq_t::input_matrix_array2_t;
	using input_matrix_array3_t = typename slq_t::input_matrix_array3_t;
	using state_input_matrix_array_t = typename slq_t::state_input_matrix_array_t;
	using state_input_matrix_array2_t = typename slq_t::state_input_matrix_array2_t;
	using constraint1_vector_array_t = typename slq_t::constraint1_vector_array_t;
	using constraint1_vector_array2_t = typename slq_t::constraint1_vector_array2_t;
	using constraint1_state_matrix_array_t = typename slq_t::constraint1_state_matrix_array_t;
	using constraint1_state_matrix_array2_t = typename slq_t::constraint1_state_matrix_array2_t;
	using constraint1_input_matrix_array_t = typename slq_t::constraint1_input_matrix_array_t;
	using constraint1_input_matrix_array2_t = typename slq_t::constraint1_input_matrix_array2_t;
	using input_constraint1_matrix_array_t = typename slq_t::input_constraint1_matrix_array_t;
	using input_constraint1_matrix_array2_t = typename slq_t::input_constraint1_matrix_array2_t;
	using constraint2_vector_array_t = typename slq_t::constraint2_vector_array_t;
	using constraint2_vector_array2_t = typename slq_t::constraint2_vector_array2_t;
	using constraint2_state_matrix_array_t = typename slq_t::constraint2_state_matrix_array_t;
	using constraint2_state_matrix_array2_t = typename slq_t::constraint2_state_matrix_array2_t;
	using dynamic_matrix_array2_t = typename slq_t::dynamic_matrix_array2_t;

	typedef std::vector<constraint1_vector_array2_t, Eigen::aligned_allocator<constraint1_vector_array2_t>> constraint1_vector_array3_t;
	typedef std::vector<constraint2_vector_array2_t, Eigen::aligned_allocator<constraint2_vector_array2_t>> constraint2_vector_array3_t;

	using controlled_system_base_t = typename slq_t::controlled_system_base_t;
	using derivatives_base_t = typename slq_t::derivatives_base_t;
	using constraint_base_t = typename slq_t::constraint_base_t;
	using cost_function_base_t = typename slq_t::cost_function_base_t;

	/**
	 * Default constructor.
	 */
	SLQ_DataCollector() = default;

	/**
	 * Constructor.
	 *
	 * @param [in] systemDynamicsPtr: The system dynamics.
	 */
	SLQ_DataCollector(
			const controlled_system_base_t* systemDynamicsPtr,
			const derivatives_base_t* systemDerivativesPtr,
			const constraint_base_t* systemConstraintsPtr,
			const cost_function_base_t* costFunctionPtr);

	/**
	 * Default destructor.
	 */
	~SLQ_DataCollector() = default;

	/**
	 * Collects the required data from SLQ instance. It uses swap method wherever it is possible.
	 *
	 * @param constSlqPtr: A pointer to the SLQ instance.
	 */
	void collect(const slq_t* constSlqPtr);

	/******************
	 * SLQ variables image
	 ******************/
	scalar_t initTime_;
	scalar_t finalTime_;
	state_vector_t initState_;

	size_t initActivePartition_;
	size_t finalActivePartition_;
	size_t numPartitions_ = 0;
	scalar_array_t partitioningTimes_;

	unsigned long long int rewindCounter_;

	scalar_array_t eventTimes_;
	size_array_t   subsystemsSequence_;

	linear_controller_array_t optimizedControllersStock_;

//	std::vector<scalar_array_t>  optimizedTimeTrajectoriesStock_;
//	std::vector<size_array_t>    optimizedEventsPastTheEndIndecesStock_;
//	state_vector_array2_t        optimizedStateTrajectoriesStock_;
//	input_vector_array2_t        optimizedInputTrajectoriesStock_;

	std::vector<scalar_array_t> nominalTimeTrajectoriesStock_;
	std::vector<size_array_t>   nominalEventsPastTheEndIndecesStock_;
	state_vector_array2_t       nominalStateTrajectoriesStock_;
	input_vector_array2_t       nominalInputTrajectoriesStock_;

	state_matrix_array2_t       AmTrajectoriesStock_;
	state_input_matrix_array2_t BmTrajectoriesStock_;

	std::vector<size_array_t>          nc1TrajectoriesStock_;  	// nc1: Number of the Type-1  active constraints
	constraint1_vector_array2_t        EvTrajectoriesStock_;
	constraint1_state_matrix_array2_t  CmTrajectoriesStock_;
	constraint1_input_matrix_array2_t  DmTrajectoriesStock_;

	std::vector<size_array_t>          nc2TrajectoriesStock_;  // nc2: Number of the Type-2 active constraints
	constraint2_vector_array2_t        HvTrajectoriesStock_;
	constraint2_state_matrix_array2_t  FmTrajectoriesStock_;
	std::vector<size_array_t>          nc2FinalStock_;
	constraint2_vector_array2_t        HvFinalStock_;
	constraint2_state_matrix_array2_t  FmFinalStock_;

	std::vector<size_array_t>   ncIneqTrajectoriesStock_;  // ncIneq: Number of inequality constraints
	scalar_array3_t       		hTrajectoryStock_;
	state_vector_array3_t       dhdxTrajectoryStock_;
	state_matrix_array3_t       ddhdxdxTrajectoryStock_;
	input_vector_array3_t       dhduTrajectoryStock_;
	input_matrix_array3_t       ddhduduTrajectoryStock_;
	input_state_matrix_array3_t ddhdudxTrajectoryStock_;

	eigen_scalar_array2_t        qFinalStock_;
	state_vector_array2_t        QvFinalStock_;
	state_matrix_array2_t        QmFinalStock_;

	eigen_scalar_array2_t        qTrajectoriesStock_;
	state_vector_array2_t        QvTrajectoriesStock_;
	state_matrix_array2_t        QmTrajectoriesStock_;
	input_vector_array2_t        RvTrajectoriesStock_;
	input_matrix_array2_t        RmTrajectoriesStock_;
	input_state_matrix_array2_t  PmTrajectoriesStock_;

	input_matrix_array2_t                RmInverseTrajectoriesStock_;
	state_matrix_array2_t                AmConstrainedTrajectoriesStock_;
	state_matrix_array2_t                QmConstrainedTrajectoriesStock_;
	state_vector_array2_t                QvConstrainedTrajectoriesStock_;
	dynamic_matrix_array2_t              RmInvConstrainedCholTrajectoryStock_;
	input_constraint1_matrix_array2_t    DmDagerTrajectoriesStock_;
	input_vector_array2_t                EvProjectedTrajectoriesStock_;  // DmDager * Ev
	input_state_matrix_array2_t          CmProjectedTrajectoriesStock_;  // DmDager * Cm
	input_matrix_array2_t                DmProjectedTrajectoriesStock_;  // DmDager * Dm

	// terminal cost which is interpreted as the Heuristic function
	eigen_scalar_t sHeuristics_;
	state_vector_t SvHeuristics_;
	state_matrix_t SmHeuristics_;

	std::vector<scalar_array_t>  SsTimeTrajectoriesStock_;
	std::vector<scalar_array_t>  SsNormalizedTimeTrajectoriesStock_;
	std::vector<size_array_t>    SsNormalizedEventsPastTheEndIndecesStock_;
	state_matrix_array2_t        SmTrajectoriesStock_;
	state_vector_array2_t        SvTrajectoriesStock_;
	state_vector_array2_t        SveTrajectoriesStock_;
	eigen_scalar_array2_t        sTrajectoriesStock_;

	/******************
	 * SLQ missing variables
	 ******************/
	state_vector_array2_t        nominalFlowMapTrajectoriesStock_;
	constraint1_vector_array3_t  EvDevEventTimesTrajectoriesStockSet_;           // state-input constraint derivative w.r.t. event times
	input_vector_array3_t        EvDevEventTimesProjectedTrajectoriesStockSet_;  // DmDager * EvDevEventTimes

protected:
	/**
	 * Resizes the data of the other-class's member.
	 *
	 * @param numPartitions: Number of partitions.
	 * @param otherPtr: A pointer to the OTHER_CLASS instance
	 * @return True if number of partitions is changed.
	 */
	void resizeDataContainer(const size_t& numPartitions);

	/**
	 * Calculates the time derivatives of the nominal state trajectory.
	 *
	 * @param [in] constSlqPtr: A pointer to the SLQ instance.
	 * @param [in] timeTrajectoriesStock: The time trajectory stamp.
	 * @param [in] stateTrajectoriesStock: The state trajectory.
	 * @param [in] inputTrajectoriesStock: The control input trajectory.
	 * @param [out] flowMapTrajectoriesStock: An array of the time
	 * derivatives of the nominal state trajectory.
	 */
	void calculateFlowMap(
			const slq_t* constSlqPtr,
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const input_vector_array2_t& inputTrajectoriesStock,
			state_vector_array2_t& flowMapTrajectoriesStock);

	/**
	 * Calculates sensitivity of the state-input constraints to event times.
	 *
	 * @param [in] constSlqPtr: A pointer to the SLQ instance.
	 * @param [in] timeTrajectoriesStock: The time trajectory stamp.
	 * @param [in] stateTrajectoriesStock: The state trajectory.
	 * @param [in] inputTrajectoriesStock: The control input trajectory.
	 * @param [out] EvDevEventTimesTrajectoriesStockSet: The sensitivity of the state-input constraints to the event times. Here
	 * EvDevEventTimesTrajectoriesStockSet[i] is the EvDevEventTimesTrajectoriesStock for the i'th event time.
	 * @param [out] EvDevEventTimesProjectedTrajectoriesStockSet: The projected sensitivity of the state-input constraints to the event times,
	 * Defined as (DmDager * EvDevEventTimes).
	 */
	void calculateStateInputConstraintsSensitivity(
			const slq_t* constSlqPtr,
			const std::vector<scalar_array_t>& timeTrajectoriesStock,
			const state_vector_array2_t& stateTrajectoriesStock,
			const input_vector_array2_t& inputTrajectoriesStock,
			constraint1_vector_array3_t& EvDevEventTimesTrajectoriesStockSet,
			input_vector_array3_t& EvDevEventTimesProjectedTrajectoriesStockSet);

private:
	std::unique_ptr<controlled_system_base_t>  systemDynamicsPtr_;
	std::unique_ptr<derivatives_base_t>        systemDerivativesPtr_;
	std::unique_ptr<constraint_base_t>         systemConstraintsPtr_;
	std::unique_ptr<cost_function_base_t>      costFunctionPtr_;

};

} // namespace ocs2

#include "implementation/SLQ_DataCollector.h"

#endif /* SLQ_DATACOLLECTOR_OCS2_H_ */
