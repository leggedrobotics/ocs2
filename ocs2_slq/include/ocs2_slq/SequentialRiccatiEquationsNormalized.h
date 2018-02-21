/*
 * SequentialRiccatiEquationsNormalized.h
 *
 *  Created on: Jan 26, 2017
 *      Author: farbod
 */

#ifndef SEQUENTIALRICCATIEQUATIONSNORMALIZED_OCS2_H_
#define SEQUENTIALRICCATIEQUATIONSNORMALIZED_OCS2_H_

#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/SystemBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * This class implements the time-normalized Riccati equations for SLQ problem.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialRiccatiEquationsNormalized : public SystemBase<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		/** If STATE_DIM=n, Then: n(n+1)/2 entries from triangular matrix Sm, n entries from vector Sv and +1 one from a scalar */
		S_DIM_ = STATE_DIM*(STATE_DIM+1)/2 + STATE_DIM + 1
	};

	typedef Eigen::Matrix<double,S_DIM_,1> s_vector_t;
	typedef std::vector<s_vector_t, Eigen::aligned_allocator<s_vector_t> > s_vector_array_t;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::controller_t controller_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::size_array_t 	size_array_t;
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

	/**
	 * Default constructor.
	 */
	SequentialRiccatiEquationsNormalized():
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

	/**
	 * Default destructor.
	 */
	~SequentialRiccatiEquationsNormalized() {}

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
	void setData(const scalar_t& switchingTimeStart, const scalar_t& switchingTimeFinal,
			const scalar_array_t* timeStampPtr,
			const state_matrix_array_t* AmPtr, const control_gain_matrix_array_t* BmPtr,
			const eigen_scalar_array_t* qPtr, const state_vector_array_t* QvPtr, const state_matrix_array_t* QmPtr,
			const control_vector_array_t* RvPtr, const control_matrix_array_t* RmInversePtr, const control_matrix_array_t* RmPtr,
			const control_feedback_array_t* PmPtr,
			const size_array_t* eventsPastTheEndIndecesPtr,
			const eigen_scalar_array_t* qFinalPtr, const state_vector_array_t* QvFinalPtr, const state_matrix_array_t* QmFianlPtr)  {

		SystemBase<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>::numFunctionCalls_ = 0;

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

		eventTime_.clear();
		eventTime_.reserve(eventsPastTheEndIndecesPtr->size());

		for (auto indexItr=eventsPastTheEndIndecesPtr->begin(); indexItr!=eventsPastTheEndIndecesPtr->end(); ++indexItr) {
			eventTime_.push_back( timeStampPtr->at(*indexItr-1) );
		}

		qFinalPtr_  = qFinalPtr;
		QvFinalPtr_ = QvFinalPtr;
		QmFianlPtr_ = QmFianlPtr;

//		std::cout << std::endl;
//		std::cout << "timeStampPtr.size(): " << timeStampPtr->size() << std::endl;
//		for (size_t k=0; k<timeStampPtr->size(); k++) {
//
//			std::cout << "Am[" << timeStampPtr->at(k) << "]: \n" << AmPtr->at(k) << std::endl;
//			std::cout << "Bm[" << timeStampPtr->at(k) << "]: \n" << BmPtr->at(k) << std::endl;
//			std::cout << "Qm[" << timeStampPtr->at(k) << "]: \n" << QmPtr->at(k) << std::endl;
//			std::cout << "Pm[" << timeStampPtr->at(k) << "]: \n" << PmPtr->at(k) << std::endl;
//			std::cout << "Rm[" << timeStampPtr->at(k) << "]: \n" << RmPtr->at(k) << std::endl;
//			std::cout << "RmInverse[" << timeStampPtr->at(k) << "]: \n" << RmInversePtr->at(k) << std::endl;
//			std::cout << std::endl;
//		}

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
	}

	/**
	 * Riccati jump map at switching moments
	 *
	 * @param [in] time: Normalized transition time
	 * @param [in] state: transition state
	 * @param [out] mappedState: mapped state after transition
	 */
	void mapState(const scalar_t& z, const s_vector_t& state, s_vector_t& mappedState) override {

		scalar_t time = switchingTimeFinal_ + (switchingTimeStart_-switchingTimeFinal_)*z;

		size_t index = find(eventTime_, time);

		if (index == eventTime_.size())
			throw std::runtime_error("The Riccati state jump time is not defined.");

		s_vector_t allSsJump;
		convert2Vector(QmFianlPtr_->at(index), QvFinalPtr_->at(index), qFinalPtr_->at(index), allSsJump);

		mappedState = state + allSsJump;
	}

	/**
	 * Computes derivatives.
	 *
	 * @param [in] z: Normalized time.
	 * @param [in] allSs: Single vector constructed by concatenating Sm, Sv and s.
	 * @param [out] derivatives: d(allSs)/dz.
	 */
	void computeDerivative(const scalar_t& z, const s_vector_t& allSs, s_vector_t& derivatives) {

		SystemBase<STATE_DIM*(STATE_DIM+1)/2+STATE_DIM+1>::numFunctionCalls_++;

		// denormalized time
		scalar_t t = switchingTimeFinal_ + (switchingTimeStart_-switchingTimeFinal_)*z;

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
		__dsdt  = __q   - 0.5 *__Lv.transpose() *__Rm * __Lv;

		// Riccati equations for the equivalent system
		__dSmdz = (switchingTimeFinal_-switchingTimeStart_)*__dSmdt;
		__dSvdz = (switchingTimeFinal_-switchingTimeStart_)*__dSvdt;
		__dsdz  = (switchingTimeFinal_-switchingTimeStart_)*__dsdt;

		convert2Vector(__dSmdz, __dSvdz, __dsdz, derivatives);

//		std::cout << ">>>> time: " << t << std::endl;
//		std::cout << "__dSmdt: \n" << __dSmdt << std::endl;
//		std::cout << "__dSvdt: \n" << __dSvdt.transpose() << std::endl;
//		std::cout << "switchingTimeFinal_-switchingTimeStart_: \n" << switchingTimeFinal_-switchingTimeStart_ << std::endl;
//		std::cout << "derivatives: \n" << derivatives.transpose() << std::endl;
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

	std::vector<double> eventTime_;
	const eigen_scalar_array_t* qFinalPtr_;
	const state_vector_array_t* QvFinalPtr_;
	const state_matrix_array_t* QmFianlPtr_;
};

}

#endif /* SEQUENTIALRICCATIEQUATIONSNORMALIZED_OCS2_H_ */
