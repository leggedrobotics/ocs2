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

#ifndef OCS2_OCS2_OCS2_H_
#define OCS2_OCS2_OCS2_H_

#include <array>
#include <memory>
#include <iterator>
#include <algorithm>

#include <ocs2_frank_wolfe/GradientDescent.h>

#include "ocs2_ocs2/upper_level_op/UpperLevelCost.h"
#include "ocs2_ocs2/upper_level_op/UpperLevelConstraints.h"

namespace ocs2{

/**
 * OCS2
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class OCS2 : public GradientDescent
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef GradientDescent BASE;
	using dynamic_vector_t = typename BASE::dynamic_vector_t;
	using dynamic_matrix_t = typename BASE::dynamic_matrix_t;

	typedef UpperLevelCost<STATE_DIM, INPUT_DIM> upper_level_cost_t;
	typedef UpperLevelConstraints upper_level_constraints_t;

	using state_vector_t           = typename upper_level_cost_t::state_vector_t;
	using controlled_system_base_t = typename upper_level_cost_t::controlled_system_base_t;
	using derivatives_base_t       = typename upper_level_cost_t::derivatives_base_t;
	using constraint_base_t        = typename upper_level_cost_t::constraint_base_t;
	using cost_function_base_t     = typename upper_level_cost_t::cost_function_base_t;
	using operating_trajectories_base_t = typename upper_level_cost_t::operating_trajectories_base_t;

	/**
	 * Default constructor.
	 */
	OCS2() = default;

	/**
	 * Constructor
	 * @param [in] subsystemDynamicsPtr
	 * @param [in] subsystemDerivativesPtr
	 * @param [in] subsystemCostFunctionsPtr
	 * @param [in] slqSettings
	 * @param [in] stateOperatingPoints
	 * @param [in] inputOperatingPoints
	 */
	OCS2(const controlled_system_base_t* systemDynamicsPtr,
			const derivatives_base_t* systemDerivativesPtr,
			const constraint_base_t* systemConstraintsPtr,
			const cost_function_base_t* costFunctionPtr,
			const operating_trajectories_base_t* operatingTrajectoriesPtr,
			const SLQ_Settings& settings = SLQ_Settings(),
			std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr,
			const cost_function_base_t* heuristicsFunctionPtr = nullptr,
			const GDDP_Settings& gddpSettings = GDDP_Settings(),
			const NLP_Settings& nlpSettings = NLP_Settings());

	/**
	 * Default destructor.
	 */
	virtual ~OCS2() = default;

	/**
	 * Gets a reference to the GDDP settings structure.
	 *
	 * @return a reference to the GDDP settings.
	 */
	GDDP_Settings& gddpSettings();

	/**
	 * Gets a reference to the SLQ settings structure.
	 *
	 * @return a reference to the SLQ settings.
	 */
	SLQ_Settings& slqSettings();

	/**
	 * The main routine of DDP which runs DDP for a given initial state, initial time, and final time. In order
	 * to retrieve the initial nominal trajectories in the forward pass, DDP will use the given operatingTrajectories
	 * in the constructor.
	 *
	 * @param [in] initTime: The initial time.
	 * @param [in] initState: The initial state.
	 * @param [in] finalTime: The final time.
	 * @param [in] partitioningTimes: The partitioning times between subsystems.
	 */
	void run(
			const scalar_t& initTime,
			const state_vector_t& initState,
			const scalar_t& finalTime,
			const scalar_array_t& partitioningTimes,
			const scalar_array_t& initEventTimes);

private:
	std::unique_ptr<upper_level_cost_t> ulCostPtr_;
	std::unique_ptr<upper_level_constraints_t> ulConstraintsPtr_;
};

}  // end of ocs2 namespace

#include "implementation/OCS2.h"

#endif /* OCS2_OCS2_OCS2_H_ */
