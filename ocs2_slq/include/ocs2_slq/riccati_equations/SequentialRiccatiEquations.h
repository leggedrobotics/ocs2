/*
 * SequentialRiccatiEquations.h
 *
 *  Created on: October 19, 2016
 *      Author: farbod
 */

#ifndef SEQUENTIALRICCATIEQUATIONS_OCS2_H_
#define SEQUENTIALRICCATIEQUATIONS_OCS2_H_

#include <Eigen/Core>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * This class implements the Riccati equations for SLQ problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialRiccatiEquations : public ODE_Base<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		/** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
		S_DIM_ = STATE_DIM*(STATE_DIM+1)/2 + STATE_DIM + 1
	};

	typedef Eigen::Matrix<double,S_DIM_,1> s_vector_t;
	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::input_vector_t 		input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::input_state_t 	  input_state_t;
	typedef typename DIMENSIONS::input_state_array_t input_state_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::input_matrix_t 		input_matrix_t;
	typedef typename DIMENSIONS::input_matrix_array_t input_matrix_array_t;
	typedef typename DIMENSIONS::state_input_matrix_t 		 state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;

	/**
	 * Default constructor.
	 */
	SequentialRiccatiEquations(const bool& useMakePSD)
	: useMakePSD_(useMakePSD)
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
	, Pm_(input_state_t::Zero())
	, dSmdt_(state_matrix_t::Zero())
	, dSmdz_(state_matrix_t::Zero())
	, dSvdt_(state_vector_t::Zero())
	, dSvdz_(state_vector_t::Zero())
	, dsdt_(eigen_scalar_t::Zero())
	, dsdz_(eigen_scalar_t::Zero())
	, Lm_(input_state_t::Zero())
	, Lv_(input_vector_t::Zero())
	, AtransposeSm_(state_matrix_t::Zero())
	, LmtransposeRm_(state_input_matrix_t::Zero())
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
			allSs.template segment(count, nRows) << Eigen::Map<const Eigen::VectorXd>(Sm.data() + nCols*STATE_DIM, nRows);
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
			Sm.template block(rows, 0, 1, nCols)  << Eigen::Map<const Eigen::VectorXd>(allSs.data()+count, nCols).transpose();
			Sm.template block(0, rows, nCols-1, 1)  << Eigen::Map<const Eigen::VectorXd>(allSs.data()+count, nCols-1); // "nCols-1" because diagonal elements have already been covered
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
			const input_state_array_t* PmPtr)  {

		ODE_Base<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>::numFunctionCalls_ = 0;

		alpha_ = learningRate;

		activeSubsystem_ = activeSubsystem;
		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;

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
	}

	/**
	 * Computes derivatives.
	 *
	 * @param [in] z: Time.
	 * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
	 * @param [out] derivatives: d(allSs)/dz.
	 */
	void computeFlowMap(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) {

		ODE_Base<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>::numFunctionCalls_++;

		// denormalized time
		scalar_t t = switchingTimeFinal_ - t;

		convert2Matrix(allSs, Sm_, Sv_, s_);

		// numerical consideration
		if(useMakePSD_==true)
			bool hasNegativeEigenValue = makePSD(Sm_);
		else
			Sm_ += state_matrix_t::Identity()*(1e-5);

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


		// Riccati equations for the original system
		Lm_ 	= RmInv_*(Pm_+Bm_.transpose()*Sm_);
		Lv_ 	= RmInv_*(Rv_+Bm_.transpose()*Sv_);

		/*note: according to some discussions on stackoverflow, it does not buy computation time if multiplications
		 * with symmetric matrices are executed using selfadjointView(). Doing the full multiplication seems to be faster
		 * because of vectorization */
		AtransposeSm_ = Am_.transpose()*Sm_;
		LmtransposeRm_ = Lm_.transpose()*Rm_;
		dSmdt_ = Qm_	+ AtransposeSm_ + AtransposeSm_.transpose() - LmtransposeRm_*Lm_;
		dSvdt_ = Qv_  + Am_.transpose()*Sv_ - LmtransposeRm_*Lv_;
		dsdt_  = q_   - 0.5*alpha_*(2.0-alpha_)*Lv_.transpose()*Rm_*Lv_;

		convert2Vector(dSmdt_, dSvdt_, dsdt_, derivatives);
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


private:
	bool useMakePSD_;
	scalar_t alpha_;

	size_t activeSubsystem_;
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;

	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > AmFunc_;
	LinearInterpolation<state_input_matrix_t,Eigen::aligned_allocator<state_input_matrix_t> > BmFunc_;

	LinearInterpolation<eigen_scalar_t,Eigen::aligned_allocator<eigen_scalar_t> > qFunc_;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > QvFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > QmFunc_;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> > RvFunc_;
	LinearInterpolation<input_matrix_t,Eigen::aligned_allocator<input_matrix_t> > RmInverseFunc_;
	LinearInterpolation<input_matrix_t,Eigen::aligned_allocator<input_matrix_t> > RmFunc_;
	LinearInterpolation<input_state_t,Eigen::aligned_allocator<input_state_t> > PmFunc_;


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
	input_state_t Pm_;
	state_matrix_t dSmdt_;
	state_matrix_t dSmdz_;
	state_vector_t dSvdt_;
	state_vector_t dSvdz_;
	eigen_scalar_t dsdt_;
	eigen_scalar_t dsdz_;
	input_state_t Lm_;
	input_vector_t Lv_;
	state_matrix_t AtransposeSm_;
	state_input_matrix_t LmtransposeRm_;
};

}

#endif /* SEQUENTIALRICCATIEQUATIONS_H_ */
