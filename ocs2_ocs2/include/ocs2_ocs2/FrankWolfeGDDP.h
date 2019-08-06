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

#ifndef PROJECTED_DDP_OCS2_H_
#define PROJECTED_DDP_OCS2_H_

#include <array>
#include <memory>
#include <iterator>
#include <algorithm>

// GNU Linear Programming Kit
#include <glpk.h>

#include <ocs2_slq/SLQ_BASE.h>
#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>

#include "ocs2_ocs2/GDDP.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class ProjectedGDDP : public GDDP<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef GDDP<STATE_DIM, INPUT_DIM> BASE;

	using typename BASE::scalar_t;
	using typename BASE::scalar_array_t;
	using typename BASE::dynamic_vector_t;
	using typename BASE::dynamic_matrix_t;
    using typename BASE::eigen_scalar_array3_t;
    using typename BASE::state_vector_array3_t;
    using typename BASE::input_vector_array3_t;
    using typename BASE::state_matrix_array3_t;
    using typename BASE::input_matrix_array3_t;
	using typename BASE::slq_data_collector_t;

	/**
     * Constructor.
     *
     * @param [in] settings: Structure containing the settings for the SLQ algorithm.
     */
    ProjectedGDDP(const GDDP_Settings& gddpSettings = GDDP_Settings());

	/**
	 * Default destructor.
	 */
	~ProjectedGDDP() = default;

	/**
	 * Runs the GSLQ to compute the gradient of the cost function w.r.t. the event times.
	 *
	 * eventTimes [in]: The event times vector.
	 * dcPtr [in]: A constant pointer to SLQ data collector which already collected the SLQ variables.
	 */
	void run(
			scalar_array_t eventTimes,
			const slq_data_collector_t* dcPtr,
			scalar_array_t& eventTimesOptimized,
			const scalar_t& maxStepSize = 1.0);

protected:
	/**
	 *
	 * @param gradient
	 * @param eventTimesOptimized
	 */
	void frankWolfeProblem(
			const dynamic_vector_t& gradient,
			scalar_array_t& eventTimesOptimized);

	/**
	 *
	 * @param numEvents
	 * @param startTime
	 * @param finalTime
	 * @param Cm
	 * @param Dv
	 */
	void eventTimesConstraint(
			const scalar_array_t& eventTimes,
			const size_t& activeEventTimeBeginIndex,
			const size_t& activeEventTimeEndIndex,
			const scalar_t& startTime,
			const scalar_t& finalTime,
			dynamic_matrix_t& Cm,
			dynamic_vector_t& Dv) const;

	/**
	 *
	 */
	void setupGLPK();

	/**
	 *
	 * @param Cm
	 * @param Dv
	 */
	void setupLP(
			const scalar_array_t& eventTimes,
			const dynamic_vector_t& gradient,
			const size_t& activeEventTimeBeginIndex,
			const size_t& activeEventTimeEndIndex,
			const dynamic_matrix_t& Cm,
			const dynamic_vector_t& Dv);

	/***********
	 * Variables
	 **********/
	std::unique_ptr<glp_prob, void(*)(glp_prob*)> lpPtr_;
};

} // namespace ocs2

#include "implementation/ProjectedGDDP.h"

#endif /* PROJECTED_DDP_OCS2_H_ */
