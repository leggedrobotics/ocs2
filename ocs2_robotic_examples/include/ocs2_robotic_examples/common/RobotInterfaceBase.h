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

#ifndef ROBOTINTERFACEBASE_OCS2_H_
#define ROBOTINTERFACEBASE_OCS2_H_

// C++
#include <string>
#include <iostream>
#include <stdlib.h>

// OCS2
#include <ocs2_core/Dimensions.h>
#include <ocs2_slq/SLQ_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>

namespace ocs2{

/**
 * This class implements an interface class to all the robotic examples.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RobotInterfaceBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		state_dim_ = STATE_DIM,
		input_dim_ = INPUT_DIM
	};

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;

	typedef typename DIMENSIONS::size_array_t   size_array_t;
	typedef typename DIMENSIONS::scalar_t       scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t        eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t  eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t        state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t  state_vector_array_t;
	typedef typename DIMENSIONS::input_vector_t        input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t  input_vector_array_t;
	typedef typename DIMENSIONS::dynamic_vector_t       dynamic_vector_t;
	typedef typename DIMENSIONS::dynamic_vector_array_t dynamic_vector_array_t;

	/**
	 * Constructor
	 */
	RobotInterfaceBase() = default;

	/**
	 * Destructor
	 */
	~RobotInterfaceBase() = default;

	/**
	 * Gets the initial state
	 *
	 * @param initialState: Initial state
	 */
	void getInitialState(state_vector_t& initialState) const;

	/**
	 * Gets SLQ settings.
	 *
	 * @return SLQ settings
	 */
	SLQ_Settings& slqSettings();

	/**
	 * Gets MPC settings.
	 *
	 * @return MPC settings
	 */
	MPC_Settings& mpcSettings();

	/**
	 * Setups all optimizers which you require.
	 *
	 * @param [in] taskFile: Task's file full path.
	 */
	virtual void setupOptimizer(const std::string& taskFile) = 0;

	/**
	 * Loads the settings from the task file.
	 *
	 * @param [in] taskFile: Task's file full path.
	 */
	virtual void loadSettings(const std::string& taskFile) = 0;

protected:
	/**
	 * Defines the time partitioning based on the task file values:
	 * "mpcTimeHorizon.timehorizon" and "mpcTimeHorizon.numPartitions".
	 * Time partitioning defines the time horizon and the number of data partitioning.
	 *
	 * @param [in] taskFile: Task's file full path.
	 * @param [out] timeHorizon: MPC time horizon.
	 * @param [out] numPartitions: The number of data partitioning.
	 * @param [out] partitioningTimes: The time partitioning.
	 * @param [in] verbose: Whether to print out the loaded variables.
	 */
	void definePartitioningTimes(
			const std::string& taskFile,
			scalar_t& timeHorizon,
			size_t& numPartitions,
			scalar_array_t& partitioningTimes,
			bool verbose = false);

	/**
	 * Loads initial state from the task file.
	 *
	 * @param [in] taskFile: Task's file full path.
	 * @param [out] initialState: Initial state.
	 */
	void loadInitialState(
			const std::string& taskFile,
			state_vector_t& initialState) const;

	/**
	 * Loads MPC time horizon and the number of data partitioning from the task file.
	 *
	 * @param [in] taskFile: Task's file full path.
	 * @param [out] timeHorizon: MPC time horizon.
	 * @param [out] numPartitions: The number of data partitioning.
	 * @param [in] verbose: Whether to print out the loaded variables.
	 */
	void loadMpcTimeHorizon(
			const std::string& taskFile,
			scalar_t& timeHorizon,
			size_t& numPartitions,
			bool verbose = false) const;

/**************
 * Variables
 **************/
	SLQ_Settings slqSettings_;
	MPC_Settings mpcSettings_;

	state_vector_t initialState_;
	input_vector_t initialInput_;

};

} // namespace ocs2

#include "implementation/RobotInterfaceBase.h"

#endif /* ROBOTINTERFACEBASE_OCS2_H_ */
