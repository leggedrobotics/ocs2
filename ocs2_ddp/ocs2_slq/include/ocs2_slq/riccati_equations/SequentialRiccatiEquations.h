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

#ifndef SEQUENTIALRICCATIEQUATIONS_OCS2_H_
#define SEQUENTIALRICCATIEQUATIONS_OCS2_H_

#include <Eigen/Core>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * This class implements the Riccati equations for SLQ problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
  */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialRiccatiEquations : public OdeBase<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		/** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
		S_DIM_ = STATE_DIM*(STATE_DIM+1)/2 + STATE_DIM + 1
	};

	typedef Eigen::Matrix<double,S_DIM_,1> s_vector_t;
	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	using controller_t = typename DIMENSIONS::controller_t;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t;
	using eigen_scalar_t = typename DIMENSIONS::eigen_scalar_t;
	using eigen_scalar_array_t = typename DIMENSIONS::eigen_scalar_array_t;
	using state_vector_t = typename DIMENSIONS::state_vector_t;
	using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
	using input_vector_t = typename DIMENSIONS::input_vector_t;
	using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
	using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
	using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
	using state_matrix_t = typename DIMENSIONS::state_matrix_t;
	using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
	using input_matrix_t = typename DIMENSIONS::input_matrix_t;
	using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
	using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
	using state_input_matrix_array_t = typename DIMENSIONS::state_input_matrix_array_t;

	/**
	 * Default constructor.
	 */
	SequentialRiccatiEquations(
			const bool& useMakePSD,
			const scalar_t& addedRiccatiDiagonal)

	: useMakePSD_(useMakePSD)
	, addedRiccatiDiagonal_(addedRiccatiDiagonal)
	, Sm_(state_matrix_t::Zero())
	, Sv_(state_vector_t::Zero())
	, s_ (eigen_scalar_t::Zero())
	, Am_(state_matrix_t::Zero())
	, Bm_(state_input_matrix_t::Zero())
	, q_ (eigen_scalar_t::Zero())
	, Qv_(state_vector_t::Zero())
	, Qm_(state_matrix_t::Zero())
	, Rv_(input_vector_t::Zero())
	, RmInv_(input_matrix_t::Zero())
	, Rm_(input_matrix_t::Zero())
	, Pm_(input_state_matrix_t::Zero())
	, dSmdt_(state_matrix_t::Zero())
	, dSmdz_(state_matrix_t::Zero())
	, dSvdt_(state_vector_t::Zero())
	, dSvdz_(state_vector_t::Zero())
	, dsdt_(eigen_scalar_t::Zero())
	, dsdz_(eigen_scalar_t::Zero())
	, Lm_(input_state_matrix_t::Zero())
	, Lv_(input_vector_t::Zero())
	, Am_transposeSm_(state_matrix_t::Zero())
	, Lm_transposeRm_(state_input_matrix_t::Zero())
	{}

	/**
	 * Default destructor.
	 */
	~SequentialRiccatiEquations() = default;

	/**
	 * Transcribe symmetric matrix Sm, vector Sv and scalar s into a single vector
	 *
	 * @param [in] Sm: \f$ S_m \f$
	 * @param [in] Sv: \f$ S_v \f$
	 * @param [in] s: \f$ s \f$
	 * @param [out] allSs: Single vector constructed by concatenating Sm, Sv and s.
	 */
	static void convert2Vector(const state_matrix_t& Sm, const state_vector_t& Sv, const eigen_scalar_t& s, s_vector_t& allSs)  {

		/*Sm is symmetric. Here, we only extract the upper triangular part and transcribe it in column-wise fashion into allSs*/
		size_t count = 0;	// count the total number of scalar entries covered
		size_t nRows = 0;
		for(size_t nCols=0; nCols < STATE_DIM; nCols++)
		{
			nRows = nCols+1;
			allSs.segment(count, nRows) << Eigen::Map<const Eigen::VectorXd>(Sm.data() + nCols*STATE_DIM, nRows);
			count += nRows;
		}

		/* add data from Sv on top*/
		allSs.template segment<STATE_DIM>((STATE_DIM*(STATE_DIM+1))/2) <<  Eigen::Map<const Eigen::VectorXd>(Sv.data(), STATE_DIM);

		/* add s as last element*/
		allSs.template tail<1> () << s;
	}

    /**
    * Transcribes the stacked vector allSs into a symmetric matrix, Sm, a vector, Sv and a single scalar, s.
    * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
	 * @param [out] Sm: \f$ S_m \f$
	 * @param [out] Sv: \f$ S_v \f$
	 * @param [out] s: \f$ s \f$
    */
	static void convert2Matrix(const s_vector_t& allSs, state_matrix_t& Sm, state_vector_t& Sv, eigen_scalar_t& s)  {

		/*Sm is symmetric. Here, we map the first entries from allSs onto the respective elements in the symmetric matrix*/
		size_t count = 0;
		size_t nCols = 0;
		for(size_t rows=0; rows < STATE_DIM; rows++)
		{
			nCols = rows+1;
			Sm.block(rows, 0, 1, nCols)  << Eigen::Map<const Eigen::VectorXd>(allSs.data()+count, nCols).transpose();
			Sm.block(0, rows, nCols-1, 1)  << Eigen::Map<const Eigen::VectorXd>(allSs.data()+count, nCols-1); // "nCols-1" because diagonal elements have already been covered
			count += nCols;
		}

		/*extract the vector Sv*/
		Sv = Eigen::Map<const Eigen::VectorXd>(allSs.data()+(STATE_DIM*(STATE_DIM+1))/2, STATE_DIM);

		/*extract s as the last element */
		s  = allSs.template tail<1>();
	}

	/**
	 * Sets coefficients of the model.
	 *
	 * @param [in] learningRate: The learning rate.
	 * @param [in] activeSubsystem: The index of the active subsystem.
	 * @param [in] switchingTimeStart: The start time of the subsystem.
	 * @param [in] switchingTimeFinal: The final time of the subsystem.
	 * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
	 * @param [in] AmPtr: A pointer to the trajectory of \f$ A_m(t) \f$ .
	 * @param [in] BmPtr: A pointer to the trajectory of \f$ B_m(t) \f$ .
	 * @param [in] qPtr: A pointer to the trajectory of \f$ q(t) \f$ .
	 * @param [in] QvPtr: A pointer to the trajectory of \f$ Q_v(t) \f$ .
	 * @param [in] QmPtr: A pointer to the trajectory of \f$ Q_m(t) \f$ .
	 * @param [in] RvPtr: A pointer to the trajectory of \f$ R_v(t) \f$ .
	 * @param [in] RmInversePtr: A pointer to the trajectory of \f$ R_m^{-1}(t) \f$ .
	 * @param [in] RmPtr: A pointer to the trajectory of \f$ R_m(t) \f$ .
	 * @param [in] PmPtr: A pointer to the trajectory of \f$ P_m(t) \f$ .
	 */
	void setData(const scalar_t& learningRate,
			const size_t& activeSubsystem, const scalar_t& switchingTimeStart, const scalar_t& switchingTimeFinal,
			const scalar_array_t* timeStampPtr,
			const state_matrix_array_t* AmPtr, const state_input_matrix_array_t* BmPtr,
			const eigen_scalar_array_t* qPtr, const state_vector_array_t* QvPtr, const state_matrix_array_t* QmPtr,
			const input_vector_array_t* RvPtr, const input_matrix_array_t* RmInversePtr, const input_matrix_array_t* RmPtr,
			const input_state_matrix_array_t* PmPtr)  {

		OdeBase<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>::numFunctionCalls_ = 0;

		alpha_ = learningRate;

		activeSubsystem_ = activeSubsystem;
		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;

		AmFunc_.setData(timeStampPtr, AmPtr);
		BmFunc_.setData(timeStampPtr, BmPtr);

		qFunc_.setData(timeStampPtr, qPtr);
		QvFunc_.setData(timeStampPtr, QvPtr);
		QmFunc_.setData(timeStampPtr, QmPtr);
		RvFunc_.setData(timeStampPtr, RvPtr);
		RmInverseFunc_.setData(timeStampPtr, RmInversePtr);
		RmFunc_.setData(timeStampPtr, RmPtr);
		PmFunc_.setData(timeStampPtr, PmPtr);
	}

	/**
	 * Computes derivatives.
	 *
	 * @param [in] z: Time.
	 * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
	 * @param [out] derivatives: d(allSs)/dz.
	 */
	void computeFlowMap(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) {

		OdeBase<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>::numFunctionCalls_++;

		// denormalized time
		scalar_t t = switchingTimeFinal_ - t;

		convert2Matrix(allSs, Sm_, Sv_, s_);

		const auto greatestLessTimeStampIndex = AmFunc_.interpolate(t, Am_);
		BmFunc_.interpolate(t, Bm_, greatestLessTimeStampIndex);
		qFunc_.interpolate(t, q_, greatestLessTimeStampIndex);
		QvFunc_.interpolate(t, Qv_, greatestLessTimeStampIndex);
		QmFunc_.interpolate(t, Qm_, greatestLessTimeStampIndex);
		RvFunc_.interpolate(t, Rv_, greatestLessTimeStampIndex);
		RmInverseFunc_.interpolate(t, RmInv_, greatestLessTimeStampIndex);
		RmFunc_.interpolate(t, Rm_, greatestLessTimeStampIndex);
		PmFunc_.interpolate(t, Pm_, greatestLessTimeStampIndex);

		// numerical consideration
		if ( useMakePSD_ ) {
			bool hasNegativeEigenValue = makePSD(Sm_);
		} else {
			Qm_ += addedRiccatiDiagonal_ * state_matrix_t::Identity();
		}

		// Riccati equations for the original system
		Pm_.noalias() += Bm_.transpose()*Sm_; // ! Pm is changed to avoid an extra temporary
		Lm_.noalias() = RmInv_*Pm_;
		Rv_.noalias() += Bm_.transpose()*Sv_; // ! Rv is changed to avoid an extra temporary
		Lv_.noalias() = RmInv_*Rv_;

		/*note: according to some discussions on stackoverflow, it does not buy computation time if multiplications
		 * with symmetric matrices are executed using selfadjointView(). Doing the full multiplication seems to be faster
		 * because of vectorization */
		/*
		 *  Expressions written base on guidelines in http://eigen.tuxfamily.org/dox/TopicWritingEfficientProductExpression.html
		 */
		Am_transposeSm_.noalias() = Am_.transpose()*Sm_.transpose();
		Lm_transposeRm_.noalias() = Lm_.transpose()*Rm_.transpose();

		// dSmdt,  Qm_ used instead of temporary
		Qm_ += Am_transposeSm_ + Am_transposeSm_.transpose();
		Qm_.noalias() -= Lm_transposeRm_*Lm_;

		// dSvdt,  Qv_ used instead of temporary
		Qv_.noalias() += Am_.transpose()*Sv_;
		Qv_.noalias() -= Lm_transposeRm_*Lv_;

		// dsdt,   q_ used instead of temporary
		q_.noalias() -= 0.5*alpha_*(2.0-alpha_)* Lv_.transpose()*Rm_ * Lv_;

		convert2Vector(Qm_, Qv_, q_, derivatives);
	}

protected:
	/**
	 * Makes the matrix PSD.
	 * @tparam Derived type.
	 * @param [out] squareMatrix: The matrix to become PSD.
	 * @return boolean
	 */
	template <typename Derived>
	static bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {

		if (squareMatrix.rows() != squareMatrix.cols()) {
			throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");
		}

		Eigen::SelfAdjointEigenSolver<Derived> eig(squareMatrix, Eigen::EigenvaluesOnly);
		Eigen::VectorXd lambda = eig.eigenvalues();

		bool hasNegativeEigenValue = false;
		for (size_t j=0; j<lambda.size() ; j++) {
			if (lambda(j) < 0.0) {
				hasNegativeEigenValue = true;
				lambda(j) = 1e-6;
			}
		}

		if (hasNegativeEigenValue) {
			eig.compute(squareMatrix, Eigen::ComputeEigenvectors);
			squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
		} else {
			squareMatrix = 0.5*(squareMatrix+squareMatrix.transpose()).eval();
		}

		return hasNegativeEigenValue;
	}


private:
	bool useMakePSD_;
	scalar_t addedRiccatiDiagonal_;
	scalar_t alpha_;

	size_t activeSubsystem_;
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;

	EigenLinearInterpolation<state_matrix_t> AmFunc_;
	EigenLinearInterpolation<state_input_matrix_t> BmFunc_;

	EigenLinearInterpolation<eigen_scalar_t> qFunc_;
	EigenLinearInterpolation<state_vector_t> QvFunc_;
	EigenLinearInterpolation<state_matrix_t> QmFunc_;
	EigenLinearInterpolation<input_vector_t> RvFunc_;
	EigenLinearInterpolation<input_matrix_t> RmInverseFunc_;
	EigenLinearInterpolation<input_matrix_t> RmFunc_;
	EigenLinearInterpolation<input_state_matrix_t> PmFunc_;


	// members required only in computeFlowMap()
	state_matrix_t Sm_;
	state_vector_t Sv_;
	eigen_scalar_t s_;
	state_matrix_t Am_;
	state_input_matrix_t Bm_;
	eigen_scalar_t q_;
	state_vector_t Qv_;
	state_matrix_t Qm_;
	input_vector_t Rv_;
	input_matrix_t RmInv_;
	input_matrix_t Rm_;
	input_state_matrix_t Pm_;
	state_matrix_t dSmdt_;
	state_matrix_t dSmdz_;
	state_vector_t dSvdt_;
	state_vector_t dSvdz_;
	eigen_scalar_t dsdt_;
	eigen_scalar_t dsdz_;
	input_state_matrix_t Lm_;
	input_vector_t Lv_;
	state_matrix_t Am_transposeSm_;
	state_input_matrix_t Lm_transposeRm_;
};

}  // namespace ocs2

#endif /* SEQUENTIALRICCATIEQUATIONS_H_ */
