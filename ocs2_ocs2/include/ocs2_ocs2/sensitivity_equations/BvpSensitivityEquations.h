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

#ifndef BVPSENSITIVITYEQUATIONS_OCS2_H_
#define BVPSENSITIVITYEQUATIONS_OCS2_H_

#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * BVP sensitivity equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
  */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BvpSensitivityEquations : public OdeBase<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef OdeBase<STATE_DIM> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t;
	using state_vector_t = typename DIMENSIONS::state_vector_t;
	using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
	using input_vector_t = typename DIMENSIONS::input_vector_t;
	using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
	using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
	using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t ;
	using state_matrix_t = typename DIMENSIONS::state_matrix_t;
	using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
	using input_matrix_t = typename DIMENSIONS::input_matrix_t;
	using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
	using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
	using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;
	using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
	using constraint1_vector_array_t = typename DIMENSIONS::constraint1_vector_array_t;
	using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
	using constraint1_state_matrix_array_t = typename DIMENSIONS::constraint1_state_matrix_array_t;

	/**
	 * Constructor.
	 */
	BvpSensitivityEquations() = default;

	/**
	 * Default destructor.
	 */
	~BvpSensitivityEquations() = default;

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual BvpSensitivityEquations<STATE_DIM, INPUT_DIM>* clone() const {

		return new BvpSensitivityEquations<STATE_DIM, INPUT_DIM>(*this);
	}

	/**
	 * Sets Data
	 */
	void setData(
			const scalar_t& switchingTimeStart,
			const scalar_t& switchingTimeFinal,
			const scalar_array_t* timeStampPtr,
			const state_matrix_array_t* AmPtr,
			const state_input_matrix_array_t* BmPtr,
			const constraint1_state_matrix_array_t* CmPtr,
			const state_matrix_array_t* AmConstrainedPtr,
			const input_state_matrix_array_t* CmProjectedPtr,
			const state_vector_array_t* QvPtr,
			const state_vector_array_t* flowMapPtr,
			const state_vector_array_t* costatePtr,
			const constraint1_vector_array_t* lagrangianPtr,
			const scalar_array_t* controllerTimeStampPtr,
			const input_state_matrix_array_t* KmConstrainedPtr,
			const state_matrix_array_t* SmPtr)  {

		BASE::resetNumFunctionCalls();

		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;
		scalingFactor_      = switchingTimeFinal - switchingTimeStart;

		AmFunc_.setData(timeStampPtr, AmPtr);
		BmFunc_.setData(timeStampPtr, BmPtr);
		CmFunc_.setData(timeStampPtr, CmPtr);
		AmConstrainedFunc_.setData(timeStampPtr, AmConstrainedPtr);
		CmProjectedFunc_.setData(timeStampPtr, CmProjectedPtr);
		QvFunc_.setData(timeStampPtr, QvPtr);
		flowMapFunc_.setData(timeStampPtr, flowMapPtr);
		costateFunc_.setData(timeStampPtr, costatePtr);
		lagrangianFunc_.setData(timeStampPtr, lagrangianPtr);
		KmConstrainedFunc_.setData(controllerTimeStampPtr, KmConstrainedPtr);
		SmFunc_.setData(controllerTimeStampPtr, SmPtr);
	}

	/**
	 * Reset the Riccati equation
	 */
	void reset() {

	}

	/**
	 * Sets the multiplier of exogenous part of the equation. It is either zero
	 * or plus-minus 1/(s_{i+1}-s_{i})
	 *
	 * @param [in] multiplier: the multiplier of exogenous part of the equation.
	 */
	void setMultiplier(const scalar_t& multiplier) {

		multiplier_ = multiplier;
	}

	/**
	 * Computes Derivative
	 * @param [in] time: Normalized transition time
	 * @param [in] Mv: transition state
	 * @param [out] dMv: mapped state after transition
	 */
	void computeFlowMap(
			const scalar_t& z,
			const state_vector_t& Mv,
			state_vector_t& dMvdz) override {

		BASE::numFunctionCalls_++;

		// denormalized time
		const scalar_t t = switchingTimeFinal_ - scalingFactor_*z;

		auto indexAlpha = AmFunc_.interpolate(t, Am_);
		BmFunc_.interpolate(indexAlpha,  Bm_);
		CmFunc_.interpolate(indexAlpha,  Cm_);
		AmConstrainedFunc_.interpolate(indexAlpha,  AmConstrained_);
		CmProjectedFunc_.interpolate(indexAlpha,  CmProjected_);
		QvFunc_.interpolate(indexAlpha,  Qv_);

		flowMapFunc_.interpolate(indexAlpha,  flowMap_);
		costateFunc_.interpolate(indexAlpha,  costate_);
		lagrangianFunc_.interpolate(indexAlpha,  lagrangian_);

		indexAlpha = KmConstrainedFunc_.interpolate(t, KmConstrained_);
		SmFunc_.interpolate(indexAlpha,  Sm_);

		// here we have used RmConstrained = (I-DmConstrained).transpose() * Rm
		// and Km = -(I-DmConstrained) \tilde{L} - CmProjected_
		dMvdt_ = (AmConstrained_ + Bm_*(CmProjected_+KmConstrained_)).transpose()*Mv +
				multiplier_*(Qv_ + Am_.transpose()*costate_ + Cm_.transpose()*lagrangian_ + Sm_*flowMap_);

		dMvdz = scalingFactor_ * dMvdt_;
	}


private:
	scalar_t switchingTimeStart_ = 0.0;
	scalar_t switchingTimeFinal_ = 1.0;
	scalar_t scalingFactor_ = 1.0;

	scalar_t multiplier_ = 0.0;

	EigenLinearInterpolation<state_matrix_t> AmFunc_;
	EigenLinearInterpolation<state_input_matrix_t> BmFunc_;
	EigenLinearInterpolation<constraint1_state_matrix_t> CmFunc_;
	EigenLinearInterpolation<state_matrix_t> AmConstrainedFunc_;
	EigenLinearInterpolation<input_state_matrix_t> CmProjectedFunc_;
	EigenLinearInterpolation<state_vector_t> QvFunc_;
	EigenLinearInterpolation<state_vector_t> flowMapFunc_;
	EigenLinearInterpolation<state_vector_t> costateFunc_;
	EigenLinearInterpolation<constraint1_vector_t> lagrangianFunc_;
	EigenLinearInterpolation<input_state_matrix_t> KmConstrainedFunc_;
	EigenLinearInterpolation<state_matrix_t> SmFunc_;

	state_matrix_t Am_;
	state_input_matrix_t Bm_;
	constraint1_state_matrix_t Cm_;
	state_matrix_t AmConstrained_;
	input_state_matrix_t CmProjected_;
	state_vector_t Qv_;
	state_vector_t flowMap_;
	state_vector_t costate_;
	constraint1_vector_t lagrangian_;
	input_state_matrix_t KmConstrained_;
	state_matrix_t Sm_;

	state_vector_t dMvdt_;
};

} // namespace ocs2

#endif /* BVPSENSITIVITYEQUATIONS_OCS2_H_ */
