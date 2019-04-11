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

#ifndef SLQ_RICCATIEQUATIONSNORMALIZED_OCS2_H_
#define SLQ_RICCATIEQUATIONSNORMALIZED_OCS2_H_

#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * This class implements the time-normalized Riccati equations for SLQ problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SLQ_RiccatiEquationsNormalized : public ODE_Base<(STATE_DIM*(STATE_DIM+1))/2 + 2*STATE_DIM + 1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		/** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv, n entries from vector Sve, and +1 one from a scalar */
		S_DIM_ = (STATE_DIM*(STATE_DIM+1))/2 + 2*STATE_DIM + 1
	};

	typedef ODE_Base<S_DIM_> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::size_array_t 	size_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::input_vector_t 		input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::input_state_matrix_t 	  input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t input_state_matrix_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::input_matrix_t 		input_matrix_t;
	typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
	typedef typename DIMENSIONS::state_input_matrix_t 		 state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;

	typedef Eigen::Matrix<scalar_t,S_DIM_,1> s_vector_t;
	typedef std::vector<s_vector_t, Eigen::aligned_allocator<s_vector_t> > s_vector_array_t;

	/**
	 * Default constructor.
	 */
	SLQ_RiccatiEquationsNormalized(
			const bool& useMakePSD,
			const scalar_t& addedRiccatiDiagonal)

	: useMakePSD_(useMakePSD)
	, addedRiccatiDiagonal_(addedRiccatiDiagonal)
	, switchingTimeStart_(0.0)
	, switchingTimeFinal_(1.0)
	, scalingFactor_(1.0)
	, Sm_(state_matrix_t::Zero())
	, Sv_(state_vector_t::Zero())
	, Sve_(state_vector_t::Zero())
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
	, Ev_(input_vector_t::Zero())
	, Cm_(input_state_matrix_t::Zero())
	, dSmdt_(state_matrix_t::Zero())
	, dSmdz_(state_matrix_t::Zero())
	, dSvdt_(state_vector_t::Zero())
	, dSvdz_(state_vector_t::Zero())
	, dSvedt_(state_vector_t::Zero())
	, dSvedz_(state_vector_t::Zero())
	, dsdt_(eigen_scalar_t::Zero())
	, dsdz_(eigen_scalar_t::Zero())
	, Lm_(input_state_matrix_t::Zero())
	, Lv_(input_vector_t::Zero())
	, Lve_(input_vector_t::Zero())
	, Am_transposeSm_(state_matrix_t::Zero())
	, Lm_transposeRm_(state_input_matrix_t::Zero())
	, Cm_transposeRm_(state_input_matrix_t::Zero())
	{}

	/**
	 * Default destructor.
	 */
	~SLQ_RiccatiEquationsNormalized() = default;

	/**
	 * Transcribe symmetric matrix Sm, vector Sv, scalar s, and vector Sv into a single vector
	 *
	 * @param [in] Sm: \f$ S_m \f$
	 * @param [in] Sv: \f$ S_v \f$
	 * @param [in] s: \f$ s \f$
	 * @param [in] Sve: \f$ S_{v,e} \f$
	 * @param [out] allSs: Single vector constructed by concatenating Sm, Sv and s.
	 */
	static void convert2Vector(
			const state_matrix_t& Sm,
			const state_vector_t& Sv,
			const eigen_scalar_t& s,
			const state_vector_t& Sve,
			s_vector_t& allSs)  {

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
		allSs.template segment<STATE_DIM>((STATE_DIM*(STATE_DIM+1))/2) << Eigen::Map<const Eigen::VectorXd>(Sv.data(), STATE_DIM);

		/* add data from e on top*/
		allSs.template segment<1>((STATE_DIM*(STATE_DIM+1))/2 + STATE_DIM) << s;

		/* add Sve as last segment*/
		allSs.template tail<STATE_DIM>() << Sve;
	}

    /**
    * Transcribes the stacked vector allSs into a symmetric matrix, Sm, a vector Sv, a single scalar s, and a vector Sve.
    *
    * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
	 * @param [out] Sm: \f$ S_m \f$
	 * @param [out] Sv: \f$ S_v \f$
	 * @param [out] s: \f$ s \f$
	 * @param [out] Sve: \f$ S_{v,e} \f$
    */
	static void convert2Matrix(
			const s_vector_t& allSs,
			state_matrix_t& Sm,
			state_vector_t& Sv,
			eigen_scalar_t& s,
			state_vector_t& Sve)  {

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

		/*extract the scalar s*/
		s  = allSs.template segment<1>((STATE_DIM*(STATE_DIM+1))/2+STATE_DIM);

		/*extract s as the last element */
		Sve = allSs.template tail<STATE_DIM>();
	}

	/**
	 * Sets coefficients of the model.
	 *
	 * @param [in] learningRate: The learning rate.
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
	 * @param [in] EvPtr: A pointer to the trajectory of \f$ E_v(t) \f$ .
	 * @param [in] CmPtr: A pointer to the trajectory of \f$ C_m(t) \f$ .
	 */
	void setData(const scalar_t& switchingTimeStart, const scalar_t& switchingTimeFinal,
			const scalar_array_t* timeStampPtr,
			const state_matrix_array_t* AmPtr, const state_input_matrix_array_t* BmPtr,
			const eigen_scalar_array_t* qPtr, const state_vector_array_t* QvPtr, const state_matrix_array_t* QmPtr,
			const input_vector_array_t* RvPtr, const input_matrix_array_t* RmInversePtr, const input_matrix_array_t* RmPtr,
			const input_state_matrix_array_t* PmPtr,
			const input_vector_array_t* EvPtr, const input_state_matrix_array_t* CmPtr,
			const size_array_t* eventsPastTheEndIndecesPtr,
			const eigen_scalar_array_t* qFinalPtr, const state_vector_array_t* QvFinalPtr, const state_matrix_array_t* QmFianlPtr)  {

		BASE::numFunctionCalls_ = 0;

		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;
		scalingFactor_      = switchingTimeFinal - switchingTimeStart;

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

		EvFunc_.setTimeStamp(timeStampPtr);
		EvFunc_.setData(EvPtr);
		CmFunc_.setTimeStamp(timeStampPtr);
		CmFunc_.setData(CmPtr);

		eventTime_.clear();
		eventTime_.reserve(eventsPastTheEndIndecesPtr->size());

		for (const size_t& pastTheEndIndex : *eventsPastTheEndIndecesPtr)
			eventTime_.push_back( timeStampPtr->at(pastTheEndIndex-1) );

		qFinalPtr_  = qFinalPtr;
		QvFinalPtr_ = QvFinalPtr;
		QmFianlPtr_ = QmFianlPtr;
	}

	/**
	 * Reset the Riccati equation
	 */
	void reset() {

		AmFunc_.reset();
		BmFunc_.reset();
		qFunc_.reset();
		QvFunc_.reset();
		QmFunc_.reset();
		RvFunc_.reset();
		RmInverseFunc_.reset();
		RmFunc_.reset();
		PmFunc_.reset();
		EvFunc_.reset();
		CmFunc_.reset();
	}

	/**
	 * Riccati jump map at switching moments
	 *
	 * @param [in] time: Normalized transition time
	 * @param [in] state: transition state
	 * @param [out] mappedState: mapped state after transition
	 */
	void computeJumpMap(const scalar_t& z, const s_vector_t& state, s_vector_t& mappedState) override {

		scalar_t time = switchingTimeFinal_ - scalingFactor_*z;

		size_t index = find(eventTime_, time);

		if (index == eventTime_.size())
			throw std::runtime_error("The Riccati state jump time is not defined.");

		s_vector_t allSsJump;
		convert2Vector(QmFianlPtr_->at(index), QvFinalPtr_->at(index), qFinalPtr_->at(index), state_vector_t::Zero(),
				allSsJump);

		mappedState = state + allSsJump;
	}

	/**
	 * Computes derivatives.
	 *
	 * @param [in] z: Normalized time.
	 * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
	 * @param [out] derivatives: d(allSs)/dz.
	 */
	void computeFlowMap(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) {

		BASE::numFunctionCalls_++;

		// denormalized time
		const scalar_t t = switchingTimeFinal_ - scalingFactor_*z;

		convert2Matrix(allSs, Sm_, Sv_, s_, Sve_);

		// numerical consideration
		if(useMakePSD_==true)
			bool hasNegativeEigenValue = makePSD(Sm_);
		else
			Sm_ += state_matrix_t::Identity()*(addedRiccatiDiagonal_);

		AmFunc_.interpolate(t, Am_);
		size_t greatestLessTimeStampIndex = AmFunc_.getGreatestLessTimeStampIndex();
		BmFunc_.interpolate(t, Bm_, greatestLessTimeStampIndex);
		qFunc_.interpolate(t, q_, greatestLessTimeStampIndex);
		QvFunc_.interpolate(t, Qv_, greatestLessTimeStampIndex);
		QmFunc_.interpolate(t, Qm_, greatestLessTimeStampIndex);
		RvFunc_.interpolate(t, Rv_, greatestLessTimeStampIndex);
		RmInverseFunc_.interpolate(t, RmInv_, greatestLessTimeStampIndex);
		RmFunc_.interpolate(t, Rm_, greatestLessTimeStampIndex);
		PmFunc_.interpolate(t, Pm_, greatestLessTimeStampIndex);
		EvFunc_.interpolate(t, Ev_, greatestLessTimeStampIndex);
		CmFunc_.interpolate(t, Cm_, greatestLessTimeStampIndex);

		// Riccati equations for the original system
		Lm_  = RmInv_*(Pm_+Bm_.transpose()*Sm_);
		Lv_  = RmInv_*(Rv_+Bm_.transpose()*Sv_);
		Lve_ = RmInv_*Bm_.transpose()*Sve_;

		/*note: according to some discussions on stackoverflow, it does not buy computation time if multiplications
		 * with symmetric matrices are executed using selfadjointView(). Doing the full multiplication seems to be faster
		 * because of vectorization */
		Am_transposeSm_ = (Sm_*Am_).transpose();
		Lm_transposeRm_ = (Rm_*Lm_).transpose();
		Cm_transposeRm_ = (Rm_*Cm_).transpose();
		dSmdt_  = Qm_  + Am_transposeSm_ + Am_transposeSm_.transpose() - Lm_transposeRm_*Lm_;
		dSvdt_  = Qv_  + Am_.transpose()*Sv_ - Lm_transposeRm_*Lv_;
		dsdt_   = q_   - 0.5 *Lv_.transpose() *Rm_ * Lv_;
		dSvedt_ = (Cm_transposeRm_-Lm_transposeRm_)*Ev_ + Am_.transpose()*Sve_ - Lm_transposeRm_*Lve_;

		// Riccati equations for the equivalent system
		dSmdz_  = scalingFactor_ * dSmdt_;
		dSvdz_  = scalingFactor_ * dSvdt_;
		dsdz_   = scalingFactor_ * dsdt_;
		dSvedz_ = scalingFactor_ * dSvedt_;

		convert2Vector(dSmdz_, dSvdz_, dsdz_, dSvedz_, derivatives);
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

		if (squareMatrix.rows() != squareMatrix.cols())
			throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

		Eigen::SelfAdjointEigenSolver<Derived> eig(squareMatrix, Eigen::EigenvaluesOnly);
		Eigen::VectorXd lambda = eig.eigenvalues();

		bool hasNegativeEigenValue = false;
		for (size_t j=0; j<lambda.size() ; j++)
			if (lambda(j) < 0.0) {
				hasNegativeEigenValue = true;
				lambda(j) = 1e-6;
			}

		if (hasNegativeEigenValue) {
			eig.compute(squareMatrix, Eigen::ComputeEigenvectors);
			squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
		} else {
			squareMatrix = 0.5*(squareMatrix+squareMatrix.transpose()).eval();
		}

		return hasNegativeEigenValue;
	}

	template<class InputIterator, class T>
	InputIterator find (InputIterator first, InputIterator last, const T& val)
	{
	  while (first!=last) {
	    if (*first==val) return first;
	    ++first;
	  }
	  return last;
	}

	/**
	 * finds the index of an element in dataArray which is equal to value (epsilone distance)
	 * @param [in] dataArray: data array
	 * @param [in] value: enquiry value
	 * @return: index
	 */
	size_t find(const std::vector<double>& dataArray, const double& value) {

		size_t index = dataArray.size();

		for (size_t i=0; i<dataArray.size(); i++)
			if (std::abs(dataArray[i]-value)<1e-5) {
				index = i;
				break;
			}

		return index;
	}

private:
	bool useMakePSD_;
	scalar_t addedRiccatiDiagonal_;
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;
	scalar_t scalingFactor_;

	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > AmFunc_;
	LinearInterpolation<state_input_matrix_t,Eigen::aligned_allocator<state_input_matrix_t> > BmFunc_;

	LinearInterpolation<eigen_scalar_t,Eigen::aligned_allocator<eigen_scalar_t> > qFunc_;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > QvFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > QmFunc_;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> > RvFunc_;
	LinearInterpolation<input_matrix_t,Eigen::aligned_allocator<input_matrix_t> > RmInverseFunc_;
	LinearInterpolation<input_matrix_t,Eigen::aligned_allocator<input_matrix_t> > RmFunc_;
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t> > PmFunc_;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> > 		  EvFunc_;
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t> > CmFunc_;

	// members required only in computeFlowMap()
	state_matrix_t Sm_;
	state_vector_t Sv_;
	state_vector_t Sve_;
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
	input_vector_t Ev_;
	input_state_matrix_t Cm_;
	state_matrix_t dSmdt_;
	state_matrix_t dSmdz_;
	state_vector_t dSvdt_;
	state_vector_t dSvdz_;
	state_vector_t dSvedt_;
	state_vector_t dSvedz_;
	eigen_scalar_t dsdt_;
	eigen_scalar_t dsdz_;
	input_state_matrix_t Lm_;
	input_vector_t     Lv_;
	input_vector_t     Lve_;
	state_matrix_t Am_transposeSm_;
	state_input_matrix_t Lm_transposeRm_;
	state_input_matrix_t Cm_transposeRm_;

	std::vector<double> eventTime_;
	const eigen_scalar_array_t* qFinalPtr_;
	const state_vector_array_t* QvFinalPtr_;
	const state_matrix_array_t* QmFianlPtr_;
};

}

#endif /* SLQ_RICCATIEQUATIONSNORMALIZED_OCS2_H_ */
