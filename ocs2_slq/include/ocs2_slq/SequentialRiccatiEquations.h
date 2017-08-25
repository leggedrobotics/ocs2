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
#include <ocs2_core/dynamics/SystemBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * Sequential Riccati Equations Class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 * @tparam OUTPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM, size_t OUTPUT_DIM>
class SequentialRiccatiEquations : public SystemBase<OUTPUT_DIM*(OUTPUT_DIM+1)/2+OUTPUT_DIM+1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/* OUPUT_DIM = n
	 * Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
	enum {
		S_DIM_ = OUTPUT_DIM*(OUTPUT_DIM+1)/2 + OUTPUT_DIM + 1
	};

	typedef Eigen::Matrix<double,S_DIM_,1> s_vector_t;
	typedef Dimensions<OUTPUT_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::eigen_scalar_t       eigen_scalar_t;
	typedef typename DIMENSIONS::eigen_scalar_array_t eigen_scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t 		control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_feedback_t 	  control_feedback_t;
	typedef typename DIMENSIONS::control_feedback_array_t control_feedback_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::control_matrix_t 		control_matrix_t;
	typedef typename DIMENSIONS::control_matrix_array_t control_matrix_array_t;
	typedef typename DIMENSIONS::control_gain_matrix_t 		 control_gain_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_array_t control_gain_matrix_array_t;

	SequentialRiccatiEquations():
		__Sm(state_matrix_t::Zero()),
		__Sv(state_vector_t::Zero()),
		__s (eigen_scalar_t::Zero()),
		__Am(state_matrix_t::Zero()),
		__Bm(control_gain_matrix_t::Zero()),
		__q (eigen_scalar_t::Zero()),
		__Qv(state_vector_t::Zero()),
		__Qm(state_matrix_t::Zero()),
		__Rv(control_vector_t::Zero()),
		__RmInv(control_matrix_t::Zero()),
		__Rm(control_matrix_t::Zero()),
		__Pm(control_feedback_t::Zero()),
		__dSmdt(state_matrix_t::Zero()),
		__dSmdz(state_matrix_t::Zero()),
		__dSvdt(state_vector_t::Zero()),
		__dSvdz(state_vector_t::Zero()),
		__dsdt(eigen_scalar_t::Zero()),
		__dsdz(eigen_scalar_t::Zero()),
		__Lm(control_feedback_t::Zero()),
		__Lv(control_vector_t::Zero()),
		__AtransposeSm(state_matrix_t::Zero()),
		__LmtransposeRm(control_gain_matrix_t::Zero())
		{}

	~SequentialRiccatiEquations() {}

    /**
    * Transcribe symmetric matrix Sm, vector Sv and scalar s into a single vector
    * @param [in] Sm
    * @param [in] Sv
    * @param [in] s
    * @param [out] allSs
    */
	static void convert2Vector(const state_matrix_t& Sm, const state_vector_t& Sv, const eigen_scalar_t& s, s_vector_t& allSs)  {

		/*Sm is symmetric. Here, we only extract the upper triangular part and transcribe it in column-wise fashion into allSs*/
		size_t count = 0;	// count the total number of scalar entries covered
		size_t nRows = 0;
		for(size_t nCols=0; nCols < OUTPUT_DIM; nCols++)
		{
			nRows = nCols+1;
			allSs.template segment(count, nRows) << Eigen::Map<const Eigen::VectorXd>(Sm.data() + nCols*OUTPUT_DIM, nRows);
			count += nRows;
		}

		/* add data from Sv on top*/
		allSs.template segment<OUTPUT_DIM>((OUTPUT_DIM*(OUTPUT_DIM+1))/2) <<  Eigen::Map<const Eigen::VectorXd>(Sv.data(), OUTPUT_DIM);

		/* add s as last element*/
		allSs.template tail<1> () << s;
	}

    /**
    * Transcribes the stacked vector allSs into a symmetric matrix, a state vector sized Sv and a single scalar
    * @param [in] allSs
    * @param [out] Sm
    * @param [out] Sv
    * @param [out] s
    */
	static void convert2Matrix(const s_vector_t& allSs, state_matrix_t& Sm, state_vector_t& Sv, eigen_scalar_t& s)  {

		/*Sm is symmetric. Here, we map the first entries from allSs onto the respective elements in the symmetric matrix*/
		size_t count = 0;
		size_t nCols = 0;
		for(size_t rows=0; rows < OUTPUT_DIM; rows++)
		{
			nCols = rows+1;
			Sm.template block(rows, 0, 1, nCols)  << Eigen::Map<const Eigen::VectorXd>(allSs.data()+count, nCols).transpose();
			Sm.template block(0, rows, nCols-1, 1)  << Eigen::Map<const Eigen::VectorXd>(allSs.data()+count, nCols-1); // "nCols-1" because diagonal elements have already been covered
			count += nCols;
		}

		/*extract the vector Sv*/
		Sv = Eigen::Map<const Eigen::VectorXd>(allSs.data()+(OUTPUT_DIM*(OUTPUT_DIM+1))/2, OUTPUT_DIM);

		/*extract s as the last element */
		s  = allSs.template tail<1>();
	}

	/**
	 * Sets Data
	 * @param [in] learningRate
	 * @param [in] activeSubsystem
	 * @param [in] switchingTimeStart
	 * @param [in] switchingTimeFinal
	 * @param [in] timeStampPtr
	 * @param [in] AmPtr
	 * @param [in] BmPtr
	 * @param [in] qPtr
	 * @param [in] QvPtr
	 * @param [in] QmPtr
	 * @param [in] RvPtr
	 * @param [in] RmInversePtr
	 * @param [in] RmPtr
	 * @param [in] PmPtr
	 */
	void setData(const scalar_t& learningRate,
			const size_t& activeSubsystem, const scalar_t& switchingTimeStart, const scalar_t& switchingTimeFinal,
			const scalar_array_t* timeStampPtr,
			const state_matrix_array_t* AmPtr, const control_gain_matrix_array_t* BmPtr,
			const eigen_scalar_array_t* qPtr, const state_vector_array_t* QvPtr, const state_matrix_array_t* QmPtr,
			const control_vector_array_t* RvPtr, const control_matrix_array_t* RmInversePtr, const control_matrix_array_t* RmPtr,
			const control_feedback_array_t* PmPtr)  {

		SystemBase<OUTPUT_DIM*(OUTPUT_DIM+1)/2+OUTPUT_DIM+1>::numFunctionCalls_ = 0;

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
	 * moved all dynamically allocated variables, are now members (higher efficiency)
	 * @param [in] z
	 * @param [in] allSs
	 * @param [out] derivatives
	 */
	void computeDerivative(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) {

		SystemBase<OUTPUT_DIM*(OUTPUT_DIM+1)/2+OUTPUT_DIM+1>::numFunctionCalls_++;

		// denormalized time
		scalar_t t = switchingTimeFinal_ - t;

		convert2Matrix(allSs, __Sm, __Sv, __s);

		// numerical consideration
//		bool hasNegativeEigenValue = makePSD(__Sm);
		__Sm += state_matrix_t::Identity()*(1e-5);

		AmFunc_.interpolate(t, __Am);
		size_t greatestLessTimeStampIndex = AmFunc_.getGreatestLessTimeStampIndex();
		BmFunc_.interpolate(t, __Bm, greatestLessTimeStampIndex);
		qFunc_.interpolate(t, __q, greatestLessTimeStampIndex);
		QvFunc_.interpolate(t, __Qv, greatestLessTimeStampIndex);
		QmFunc_.interpolate(t, __Qm, greatestLessTimeStampIndex);
		RvFunc_.interpolate(t, __Rv, greatestLessTimeStampIndex);
		RmInverseFunc_.interpolate(t, __RmInv, greatestLessTimeStampIndex);
		RmFunc_.interpolate(t, __Rm, greatestLessTimeStampIndex);
		PmFunc_.interpolate(t, __Pm, greatestLessTimeStampIndex);


		// Riccati equations for the original system
		__Lm 	= __RmInv*(__Pm+__Bm.transpose()*__Sm);
		__Lv 	= __RmInv*(__Rv+__Bm.transpose()*__Sv);

		/*note: according to some discussions on stackoverflow, it does not buy computation time if multiplications
		 * with symmetric matrices are executed using selfadjointView(). Doing the full multiplication seems to be faster
		 * because of vectorization */
		__AtransposeSm = __Am.transpose()*__Sm;
		__LmtransposeRm = __Lm.transpose()*__Rm;
		__dSmdt = __Qm	+ __AtransposeSm + __AtransposeSm.transpose() - __LmtransposeRm*__Lm;
		__dSvdt = __Qv  + __Am.transpose()*__Sv - __LmtransposeRm*__Lv;
		__dsdt  = __q   - 0.5*alpha_*(2.0-alpha_)*__Lv.transpose()*__Rm*__Lv;

		convert2Vector(__dSmdt, __dSvdt, __dsdt, derivatives);
	}

protected:
	/**
	 *
	 * @tparam Derived
	 * @param [out] squareMatrix
	 * @return boolean
	 */
	template <typename Derived>
	static bool makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {

		if (squareMatrix.rows() != squareMatrix.cols())  throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

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

		//		Eigen::LDLT<Derived> ldlt(squareMatrix);
		//		Derived squareMatrixNew = ldlt.matrixLDLT();
		//		if (squareMatrix.isApprox(squareMatrixNew,1e-4))
		//			std::cout << ">>>>>>>>>>>>>>> Cholesky is wrong" << std::endl;
		//		if (hasNegativeEigenValue)
		//			std::cout << "lambda: " << eig.eigenvalues().head(5).transpose() << std::endl;

		return hasNegativeEigenValue;
	}


private:
	scalar_t alpha_;

	size_t activeSubsystem_;
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;

	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > AmFunc_;
	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc_;

	LinearInterpolation<eigen_scalar_t,Eigen::aligned_allocator<eigen_scalar_t> > qFunc_;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > QvFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > QmFunc_;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> > RvFunc_;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> > RmInverseFunc_;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> > RmFunc_;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > PmFunc_;


	// members required only in computeDerivative()
	state_matrix_t __Sm;
	state_vector_t __Sv;
	eigen_scalar_t __s;
	state_matrix_t __Am;
	control_gain_matrix_t __Bm;
	eigen_scalar_t __q;
	state_vector_t __Qv;
	state_matrix_t __Qm;
	control_vector_t __Rv;
	control_matrix_t __RmInv;
	control_matrix_t __Rm;
	control_feedback_t __Pm;
	state_matrix_t __dSmdt;
	state_matrix_t __dSmdz;
	state_vector_t __dSvdt;
	state_vector_t __dSvdz;
	eigen_scalar_t __dsdt;
	eigen_scalar_t __dsdz;
	control_feedback_t __Lm;
	control_vector_t __Lv;
	state_matrix_t __AtransposeSm;
	control_gain_matrix_t __LmtransposeRm;
};

}

#endif /* SEQUENTIALRICCATIEQUATIONS_H_ */
