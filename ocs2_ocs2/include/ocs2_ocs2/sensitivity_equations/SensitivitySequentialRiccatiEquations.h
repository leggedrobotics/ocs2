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

#ifndef SENSITIVITYSEQUENTIALRICCATIEQUATIONS_OCS2_H_
#define SENSITIVITYSEQUENTIALRICCATIEQUATIONS_OCS2_H_

#include <array>
#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * Sensitivity sequential Riccati equations class
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SensitivitySequentialRiccatiEquations : public ODE_Base<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		/** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
		S_DIM_ = STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1
	};

	typedef ODE_Base<S_DIM_> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::scalar_t       scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
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
	typedef typename DIMENSIONS::state_input_matrix_t       state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;
	typedef typename DIMENSIONS::dynamic_vector_t dynamic_vector_t;

	typedef Eigen::Matrix<scalar_t, S_DIM_,1> s_vector_t;
	typedef std::vector<s_vector_t, Eigen::aligned_allocator<s_vector_t> > s_vector_array_t;

	/**
	 * Default constructor.
	 */
	SensitivitySequentialRiccatiEquations() = default;

	/**
	 * Default destructor.
	 */
	~SensitivitySequentialRiccatiEquations() = default;

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM>* clone() const {

		return new SensitivitySequentialRiccatiEquations<STATE_DIM, INPUT_DIM>(*this);
	}

	/**
	 * Transcribe symmetric matrix nabla_Sm, vector nabla_Sv and
	 * scalar nabla_s into a single vector.
	 *
	 * @param [in] nabla_Sm: \f$ \partial S_m \f$
	 * @param [in] nabla_Sv: \f$ \partial S_v \f$
	 * @param [in] nabla_s: \f$ \partial s \f$
	 * @param [out] allSs: Single vector constructed by concatenating
	 * nabla_Sm, nabla_Sv and nabla_s.
	 */
	static void convert2Vector(
			const state_matrix_t& nabla_Sm,
			const state_vector_t& nabla_Sv,
			const eigen_scalar_t& nabla_s,
			s_vector_t& allSs)  {

		/* nabla_Sm is symmetric. Here, we only extract the upper triangular part and transcribe it in column-wise fashion into allSs*/
		size_t count = 0;	// count the total number of scalar entries covered
		size_t nRows = 0;
		for(size_t nCols=0; nCols < STATE_DIM; nCols++) {
			nRows = nCols+1;
			allSs.template segment(count, nRows) <<
					Eigen::Map<const dynamic_vector_t>(nabla_Sm.data() + nCols*STATE_DIM, nRows);
			count += nRows;
		}

		/* add data from nabla_Sv on top*/
		allSs.template segment<STATE_DIM>((STATE_DIM*(STATE_DIM+1))/2) <<
				Eigen::Map<const dynamic_vector_t>(nabla_Sv.data(), STATE_DIM);

		/* add nabla_s as last element*/
		allSs.template tail<1> () << nabla_s;
	}

	/**
	 * Transcribes the stacked vector allSs into a symmetric matrix, nabla_Sm,
	 * a vector, nabla_Sv and a single scalar, nabla_s.
	 *
	 * @param [in] allSs: Single vector constructed by concatenating nabla_Sm,
	 * nabla_Sv and nabla_s.
	 * @param [out] nabla_Sm: \f$ \partial S_m \f$
	 * @param [out] nabla_Sv: \f$ \partial S_v \f$
	 * @param [out] nabla_s: \f$ \partial s \f$
	 */
	static void convert2Matrix(
			const s_vector_t& allSs,
			state_matrix_t& nabla_Sm,
			state_vector_t& nabla_Sv,
			eigen_scalar_t& nabla_s)  {

		/* Sm is symmetric. Here, we map the first entries from allSs onto the respective elements in the symmetric matrix*/
		size_t count = 0;
		size_t nCols = 0;
		for(size_t rows=0; rows < STATE_DIM; rows++)
		{
			nCols = rows+1;
			nabla_Sm.template block(rows, 0, 1, nCols)  <<
					Eigen::Map<const dynamic_vector_t>(allSs.data()+count, nCols).transpose();
			// "nCols-1" because diagonal elements have already been covered
			nabla_Sm.template block(0, rows, nCols-1, 1)  <<
					Eigen::Map<const dynamic_vector_t>(allSs.data()+count, nCols-1);
			count += nCols;
		}

		/* extract the vector Sv*/
		nabla_Sv = Eigen::Map<const dynamic_vector_t>(
				allSs.data()+(STATE_DIM*(STATE_DIM+1))/2, STATE_DIM);

		/* extract s as the last element */
		nabla_s = allSs.template tail<1>();
	}

    /**
     * Sets data
     */
	void setData(
			const scalar_t& learningRate,
			const scalar_t& switchingTimeStart,
			const scalar_t& switchingTimeFinal,
			const scalar_array_t* SsTimePtr,
			const state_matrix_array_t* SmPtr,
			const state_vector_array_t* SvPtr,
			const scalar_array_t* timeStampPtr,
			const state_matrix_array_t* AmPtr,
			const state_input_matrix_array_t* BmPtr,
			const eigen_scalar_array_t* qPtr,
			const state_vector_array_t* QvPtr,
			const state_matrix_array_t* QmPtr,
			const input_vector_array_t* RvPtr,
			const input_matrix_array_t* RmInversePtr,
			const input_matrix_array_t* RmPtr,
			const input_state_matrix_array_t* PmPtr,
			const eigen_scalar_array_t* nablaqPtr,
			const state_vector_array_t* nablaQvPtr,
			const input_vector_array_t* nablaRvPtr)  {

		BASE::resetNumFunctionCalls();

		alpha_ = learningRate;

		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;
		scalingFactor_      = switchingTimeFinal - switchingTimeStart;

		SvFunc_.setTimeStamp(SsTimePtr);
		SvFunc_.setData(SvPtr);
		SmFunc_.setTimeStamp(SsTimePtr);
		SmFunc_.setData(SmPtr);

		AmFunc_.setTimeStamp(timeStampPtr);
		AmFunc_.setData(AmPtr);
		BmFunc_.setTimeStamp(timeStampPtr);
		BmFunc_.setData(BmPtr);

		qFunc_.setTimeStamp(timeStampPtr);
		qFunc_.setData(qPtr);
		QvFunc_.setTimeStamp(timeStampPtr);
		QvFunc_.setData(QvPtr);
		QmFunc_.setTimeStamp(timeStampPtr);
		QmFunc_.setData(QmPtr);
		RvFunc_.setTimeStamp(timeStampPtr);
		RvFunc_.setData(RvPtr);
		RmInverseFunc_.setTimeStamp(timeStampPtr);
		RmInverseFunc_.setData(RmInversePtr);
		RmFunc_.setTimeStamp(timeStampPtr);
		RmFunc_.setData(RmPtr);
		PmFunc_.setTimeStamp(timeStampPtr);
		PmFunc_.setData(PmPtr);

		nabla_qFunc_.setTimeStamp(timeStampPtr);
		nabla_qFunc_.setData(nablaqPtr);
		nabla_QvFunc_.setTimeStamp(timeStampPtr);
		nabla_QvFunc_.setData(nablaQvPtr);
		nabla_RvFunc_.setTimeStamp(timeStampPtr);
		nabla_RvFunc_.setData(nablaRvPtr);
	}

	/**
	 * Reset the sensitivity Riccati equation
	 */
	void reset() {

		SvFunc_.reset();
		SmFunc_.reset();
		AmFunc_.reset();
		BmFunc_.reset();
		qFunc_.reset();
		QvFunc_.reset();
		QmFunc_.reset();
		RvFunc_.reset();
		RmInverseFunc_.reset();
		RmFunc_.reset();
		PmFunc_.reset();
		nabla_qFunc_.reset();
		nabla_QvFunc_.reset();
		nabla_RvFunc_.reset();
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
     * Computes derivative
     * @param [in] z
     * @param [in] allSs
     * @param [out] derivatives
     */
	void computeFlowMap(
			const scalar_t& z,
			const s_vector_t& allSs,
			s_vector_t& derivatives)  {

		BASE::numFunctionCalls_++;

		// denormalized time
		const scalar_t t = switchingTimeFinal_ - scalingFactor_*z;

		convert2Matrix(allSs, nabla_Sm_, nabla_Sv_, nabla_s_);

		SvFunc_.interpolate(t, Sv_);
		size_t greatestLessTimeStampIndex = SvFunc_.getGreatestLessTimeStampIndex();
		SmFunc_.interpolate(t, Sm_, greatestLessTimeStampIndex);

		AmFunc_.interpolate(t, Am_);
		greatestLessTimeStampIndex = AmFunc_.getGreatestLessTimeStampIndex();
		BmFunc_.interpolate(t, Bm_, greatestLessTimeStampIndex);
		qFunc_.interpolate(t, q_);
		QvFunc_.interpolate(t, Qv_, greatestLessTimeStampIndex);
		QmFunc_.interpolate(t, Qm_, greatestLessTimeStampIndex);
		RvFunc_.interpolate(t, Rv_, greatestLessTimeStampIndex);
		RmInverseFunc_.interpolate(t, invRm_, greatestLessTimeStampIndex);
		RmFunc_.interpolate(t, Rm_, greatestLessTimeStampIndex);
		PmFunc_.interpolate(t, Pm_, greatestLessTimeStampIndex);
		nabla_qFunc_.interpolate(t, nabla_q_, greatestLessTimeStampIndex);
		nabla_QvFunc_.interpolate(t, nabla_Qv_, greatestLessTimeStampIndex);
		nabla_RvFunc_.interpolate(t, nabla_Rv_, greatestLessTimeStampIndex);

		Lv_ = invRm_ * (Rv_ + Bm_.transpose()*Sv_);
		Lm_ = invRm_ * (Pm_ + Bm_.transpose()*Sm_);

		nabla_Lv_ = invRm_ * (nabla_Rv_ + Bm_.transpose()*nabla_Sv_);
		nabla_Lm_ = invRm_ * Bm_.transpose() * nabla_Sm_;

		// Riccati equations
		if (std::abs(multiplier_) > 1e-9) {
			dSmdt_ = Qm_ + Am_.transpose()*Sm_ + Sm_.transpose()*Am_ - Lm_.transpose()*Rm_*Lm_;
			dSmdt_ = 0.5*(dSmdt_+dSmdt_.transpose()).eval();
			dSvdt_ = Qv_ + Am_.transpose()*Sv_ - Lm_.transpose()*Rm_*Lv_;
			dsdt_  = q_ - 0.5*alpha_*(2.0-alpha_)*Lv_.transpose()*Rm_*Lv_;

		} else {
			dSmdt_.setZero();
			dSvdt_.setZero();
			dsdt_.setZero();
		}

		// derivatives of Riccati equations
		nabla_dSmdt_ = Am_.transpose()*nabla_Sm_ + nabla_Sm_.transpose()*Am_
				- nabla_Lm_.transpose()*Rm_*Lm_ - Lm_.transpose()*Rm_*nabla_Lm_;
		nabla_dSmdt_ = 0.5*(nabla_dSmdt_+nabla_dSmdt_.transpose()).eval();

		nabla_dSvdt_ = nabla_Qv_ + Am_.transpose()*nabla_Sv_
				- nabla_Lm_.transpose()*Rm_*Lv_ - Lm_.transpose()*Rm_*nabla_Lv_;

		nabla_dsdt_  = nabla_q_ - 0.5*alpha_*(2-alpha_) *
				(nabla_Lv_.transpose()*Rm_*Lv_ + Lv_.transpose()*Rm_*nabla_Lv_);

		// switching time gradient for the equivalent system
		nabla_dSmdz_ = scalingFactor_ * (nabla_dSmdt_ + multiplier_*dSmdt_);
		nabla_dSvdz_ = scalingFactor_ * (nabla_dSvdt_ + multiplier_*dSvdt_);
		nabla_dsdz_  = scalingFactor_ * (nabla_dsdt_ + multiplier_*dsdt_);

		convert2Vector(nabla_dSmdz_, nabla_dSvdz_, nabla_dsdz_, derivatives);
	}


private:
	scalar_t alpha_ = 0.0;
	scalar_t switchingTimeStart_ = 0.0;
	scalar_t switchingTimeFinal_ = 1.0;
	scalar_t scalingFactor_ = 1.0;

	scalar_t multiplier_ = 0.0;

	state_matrix_t nabla_Sm_;
	state_vector_t nabla_Sv_;
	eigen_scalar_t nabla_s_;
	state_vector_t Sv_;
	state_matrix_t Sm_;
	state_matrix_t Am_;
	state_input_matrix_t Bm_;
	eigen_scalar_t q_;
	state_vector_t Qv_;
	state_matrix_t Qm_;
	input_vector_t Rv_;
	input_matrix_t invRm_;
	input_matrix_t Rm_;
	input_state_matrix_t Pm_;
	eigen_scalar_t nabla_q_;
	state_vector_t nabla_Qv_;
	input_vector_t nabla_Rv_;

	input_vector_t Lv_;
	input_state_matrix_t Lm_;
	input_vector_t nabla_Lv_;
	input_state_matrix_t nabla_Lm_;

	state_matrix_t dSmdt_;
	state_vector_t dSvdt_;
	eigen_scalar_t dsdt_;

	// normalized derivatives of Riccati equations
	state_matrix_t nabla_dSmdz_;
	state_vector_t nabla_dSvdz_;
	eigen_scalar_t nabla_dsdz_;

	// derivatives of Riccati equations
	state_matrix_t nabla_dSmdt_;
	state_vector_t nabla_dSvdt_;
	eigen_scalar_t nabla_dsdt_;

	EigenLinearInterpolation<state_vector_t> SvFunc_;
	EigenLinearInterpolation<state_matrix_t> SmFunc_;

	EigenLinearInterpolation<state_matrix_t> AmFunc_;
	EigenLinearInterpolation<state_input_matrix_t> BmFunc_;
	EigenLinearInterpolation<eigen_scalar_t> qFunc_;
	EigenLinearInterpolation<state_vector_t> QvFunc_;
	EigenLinearInterpolation<state_matrix_t> QmFunc_;
	EigenLinearInterpolation<input_vector_t> RvFunc_;
	EigenLinearInterpolation<input_matrix_t> RmInverseFunc_;
	EigenLinearInterpolation<input_matrix_t> RmFunc_;
	EigenLinearInterpolation<input_state_matrix_t> PmFunc_;

	EigenLinearInterpolation<eigen_scalar_t> nabla_qFunc_;
	EigenLinearInterpolation<state_vector_t> nabla_QvFunc_;
	EigenLinearInterpolation<input_vector_t> nabla_RvFunc_;

};

} // namespace ocs2



#endif /* SENSITIVITYSEQUENTIALRICCATIEQUATIONS_OCS2_H_ */
