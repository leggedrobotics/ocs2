/*
 * SequentialErrorEquationNormalized.h
 *
 *  Created on: Jan 26, 2017
 *      Author: farbod
 */

#ifndef SEQUENTIALERROREQUATIONNORMALIZED_OCS2_H_
#define SEQUENTIALERROREQUATIONNORMALIZED_OCS2_H_

#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/SystemBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialErrorEquationNormalized : public SystemBase<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;

	SequentialErrorEquationNormalized() {}
	~SequentialErrorEquationNormalized() {}

	void setData(const size_t& activeSubsystem, const scalar_t& switchingTimeStart, const scalar_t& switchingTimeFinal,
			scalar_array_t* const timeStampPtr, state_vector_array_t* const GvPtr, state_matrix_array_t* const GmPtr)  {

		activeSubsystem_ = activeSubsystem;
		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;

		GvFunc_.setTimeStamp(timeStampPtr);
		GvFunc_.setData(GvPtr);
		GmFunc_.setTimeStamp(timeStampPtr);
		GmFunc_.setData(GmPtr);
	}

	void computeDerivative(const scalar_t& z, const state_vector_t& Sve, state_vector_t& derivatives) {

		// denormalized time
		scalar_t t = switchingTimeFinal_ - (switchingTimeFinal_-switchingTimeStart_)*z;

		GvFunc_.interpolate(t, __Gv);
		size_t greatestLessTimeStampIndex = GvFunc_.getGreatestLessTimeStampIndex();
		GmFunc_.interpolate(t, __Gm, greatestLessTimeStampIndex);

		// Error equation for the equivalent system
		derivatives = (switchingTimeFinal_-switchingTimeStart_)*(__Gm.transpose()*Sve+__Gv);
	}


private:
	size_t activeSubsystem_;
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;

	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > GvFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > GmFunc_;

	// members required in computeDerivative
	state_vector_t __Gv;
	state_matrix_t __Gm;
};

} // namespace ocs2



#endif /* SEQUENTIALERROREQUATIONNORMALIZED_OCS2_H_ */
