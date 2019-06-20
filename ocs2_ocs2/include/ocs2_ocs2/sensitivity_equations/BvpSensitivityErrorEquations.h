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

#ifndef BVPSENSITIVITYERROREQUATIONS_OCS2_H_
#define BVPSENSITIVITYERROREQUATIONS_OCS2_H_

#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * BVP sensitivity error equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BvpSensitivityErrorEquations : public ODE_Base<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ODE_Base<STATE_DIM> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t       scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t       state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::input_vector_t       input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::input_state_matrix_t       input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t input_state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_t       state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::input_matrix_t       input_matrix_t;
	typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
	typedef typename DIMENSIONS::state_input_matrix_t 		state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_vector_t       constraint1_vector_t;
	typedef typename DIMENSIONS::constraint1_vector_array_t constraint1_vector_array_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t       constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array_t constraint1_state_matrix_array_t;
	typedef typename DIMENSIONS::dynamic_matrix_t dynamic_matrix_t;
	typedef typename DIMENSIONS::dynamic_matrix_array_t dynamic_matrix_array_t;

	/**
	 * Constructor.
	 */
	BvpSensitivityErrorEquations() = default;

	/**
	 * Default destructor.
	 */
	~BvpSensitivityErrorEquations() = default;

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual BvpSensitivityErrorEquations<STATE_DIM, INPUT_DIM>* clone() const {

		return new BvpSensitivityErrorEquations<STATE_DIM, INPUT_DIM>(*this);
	}

	/**
	 * Sets Data
	 */
	void setData(
			const scalar_t& switchingTimeStart,
			const scalar_t& switchingTimeFinal,
			const scalar_array_t* timeStampPtr,
			const state_input_matrix_array_t* BmPtr,
			const state_matrix_array_t* AmConstrainedPtr,
			const input_state_matrix_array_t* CmProjectedPtr,
			const input_state_matrix_array_t* PmPtr,
			const input_matrix_array_t* RmPtr,
			const input_matrix_array_t* RmInversePtr,
			const dynamic_matrix_array_t *RinvCholPtr,
			const input_vector_array_t* EvDevProjectedPtr,
			const scalar_array_t* SmTimeStampPtr,
			const state_matrix_array_t* SmPtr)  {


		BASE::resetNumFunctionCalls();

		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;
		scalingFactor_      = switchingTimeFinal - switchingTimeStart;

		BmFunc_.setData(timeStampPtr, BmPtr);
		AmConstrainedFunc_.setData(timeStampPtr, AmConstrainedPtr);
		CmProjectedFunc_.setData(timeStampPtr, CmProjectedPtr);
		PmFunc_.setData(timeStampPtr, PmPtr);
		RmFunc_.setData(timeStampPtr, RmPtr);
		RmInverseFunc_.setData(timeStampPtr, RmInversePtr);
		RinvChol_Func_.setData(timeStampPtr, RinvCholPtr);
		EvDevProjectedFunc_.setData(timeStampPtr, EvDevProjectedPtr);
		SmFunc_.setData(timeStampPtr, SmPtr);
	}

	/**
	 * Reset the Riccati equation
	 */
	void reset() {

	}

	/**
	 * Computes Derivative
	 * @param [in] time: Normalized transition time
	 * @param [in] Mv: transition state
	 * @param [out] dMv: mapped state after transition
	 */
	void computeFlowMap(
			const scalar_t& z,
			const state_vector_t& Mve,
			state_vector_t& dMvedz) override {

		BASE::numFunctionCalls_++;

		// denormalized time
		const scalar_t t = switchingTimeFinal_ - scalingFactor_*z;

		auto greatestLessTimeStampIndex = BmFunc_.interpolate(t, Bm_);
		AmConstrainedFunc_.interpolate(t, AmConstrained_, greatestLessTimeStampIndex);
		CmProjectedFunc_.interpolate(t, CmProjected_, greatestLessTimeStampIndex);
		PmFunc_.interpolate(t, Pm_, greatestLessTimeStampIndex);
		RmFunc_.interpolate(t, Rm_, greatestLessTimeStampIndex);
		RmInverseFunc_.interpolate(t, RmInverse_, greatestLessTimeStampIndex);
		RinvChol_Func_.interpolate(t, RinvChol_, greatestLessTimeStampIndex);
		EvDevProjectedFunc_.interpolate(t, EvDevProjected_, greatestLessTimeStampIndex);

		SmFunc_.interpolate(t, Sm_);

		// Lm
		// TODO: Double check if equations are correct after change to cholesky decomposition approach
		Lm_ = RinvChol_.transpose() *(Pm_+Bm_.transpose()*Sm_);

		dMvedt_ = (AmConstrained_ - Bm_*RinvChol_*Lm_).transpose()*Mve +
				(CmProjected_-RinvChol_*Lm_).transpose()*Rm_*EvDevProjected_;

		dMvedz = scalingFactor_ * dMvedt_;
	}


private:
	scalar_t switchingTimeStart_ = 0.0;
	scalar_t switchingTimeFinal_ = 1.0;
	scalar_t scalingFactor_ = 1.0;

	scalar_t multiplier_ = 0.0;

	LinearInterpolation<state_input_matrix_t,Eigen::aligned_allocator<state_input_matrix_t>> BmFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t>> AmConstrainedFunc_;
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t>> CmProjectedFunc_;
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t>> PmFunc_;
	LinearInterpolation<input_matrix_t,Eigen::aligned_allocator<input_matrix_t>> RmFunc_;
  	LinearInterpolation<input_matrix_t,Eigen::aligned_allocator<input_matrix_t>> RmInverseFunc_;
  	EigenLinearInterpolation <dynamic_matrix_t> RinvChol_Func_;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t>> EvDevProjectedFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t>> SmFunc_;

	state_input_matrix_t Bm_;
	state_matrix_t AmConstrained_;
	input_state_matrix_t CmProjected_;
	input_state_matrix_t Pm_;
	input_matrix_t Rm_;
	input_matrix_t RmInverse_;
  	dynamic_matrix_t RinvChol_;
	input_vector_t EvDevProjected_;
	state_matrix_t Sm_;
  	dynamic_matrix_t Lm_;

	state_vector_t dMvedt_;
};

} // namespace ocs2

#endif /* BVPSENSITIVITYERROREQUATIONS_OCS2_H_ */
