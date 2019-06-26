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

#ifndef SEQUENTIALERROREQUATIONNORMALIZED_OCS2_H_
#define SEQUENTIALERROREQUATIONNORMALIZED_OCS2_H_

#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {

/**
 * This class implements the time-normalized Error equation of Riccati equation.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialErrorEquationNormalized final : public ODE_Base<STATE_DIM> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE = ODE_Base<STATE_DIM>;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t;
	using state_vector_t = typename DIMENSIONS::state_vector_t;
	using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
	using state_matrix_t = typename DIMENSIONS::state_matrix_t;
	using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;

	/**
	 * Default constructor.
	 */
	SequentialErrorEquationNormalized() = default;

	/**
	 * Default destructor.
	 */
	~SequentialErrorEquationNormalized() = default;

	/**
	 * Sets coefficients of the model.
	 *
	 * @param [in] switchingTimeStart: The start time of the subsystem.
	 * @param [in] switchingTimeFinal: The final time of the subsystem.
	 * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
	 * @param [in] GvPtr: A pointer to the trajectory of \f$ G_v(t) \f$ .
	 * @param [in] GmPtr: A pointer to the trajectory of \f$ G_m(t) \f$ .
	 */
	void setData(
			const scalar_t& switchingTimeStart,
			const scalar_t& switchingTimeFinal,
			scalar_array_t* const timeStampPtr,
			state_vector_array_t* const GvPtr,
			state_matrix_array_t* const GmPtr) {

		BASE::resetNumFunctionCalls();

		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;

		GvFunc_.setData(timeStampPtr, GvPtr);
		GmFunc_.setData(timeStampPtr, GmPtr);
	}

	/**
	 * Error Riccati jump map at switching moments
	 *
	 * @param [in] time: Normalized transition time
	 * @param [in] state: transition state
	 * @param [out] mappedState: mapped state after transition
	 */
	void computeJumpMap(const scalar_t& z, const state_vector_t& state, state_vector_t& mappedState) override {

		mappedState = state;
	}

	/**
	 * Computes derivatives.
	 *
	 * @param [in] z: Normalized time.
	 * @param [in] Sve: Current Sve.
	 * @param [out] derivatives: d(Sve)/dz
	 */
	void computeFlowMap(const scalar_t& z, const state_vector_t& Sve, state_vector_t& derivatives) {
		BASE::numFunctionCalls_++;

		// denormalized time
		const scalar_t t = switchingTimeFinal_ + (switchingTimeStart_ - switchingTimeFinal_) * z;

		const auto greatestLessTimeStampIndex = GvFunc_.interpolate(t, Gv_);
		GmFunc_.interpolate(t, Gm_, greatestLessTimeStampIndex);

		// Error equation for the equivalent system
		derivatives = (switchingTimeFinal_ - switchingTimeStart_) * (Gm_.transpose() * Sve + Gv_);
	}

private:
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;

	LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > GvFunc_;
	LinearInterpolation<state_matrix_t, Eigen::aligned_allocator<state_matrix_t> > GmFunc_;

	// members required in computeFlowMap
	state_vector_t Gv_;
	state_matrix_t Gm_;
};

}  // namespace ocs2

#endif /* SEQUENTIALERROREQUATIONNORMALIZED_OCS2_H_ */
