/*
 * SequentialErrorEquation.h
 *
 *  Created on: Apr 13, 2016
 *      Author: farbod
 */

#ifndef SEQUENTIALERROREQUATION_OCS2_H_
#define SEQUENTIALERROREQUATION_OCS2_H_

#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/SystemBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * This class implements the Error equation of Riccati equation.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialErrorEquation : public SystemBase<STATE_DIM>
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

	/**
	 * Default constructor.
	 */
	SequentialErrorEquation() {}

	/**
	 * Default destructor.
	 */
	~SequentialErrorEquation() {}

	/**
	 * Sets coefficients of the model.
	 *
	 * @param [in] activeSubsystem: The index of the active subsystem.
	 * @param [in] switchingTimeStart: The start time of the subsystem.
	 * @param [in] switchingTimeFinal: The final time of the subsystem.
	 * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
	 * @param [in] GvPtr: A pointer to the trajectory of \f$ G_v(t) \f$ .
	 * @param [in] GmPtr: A pointer to the trajectory of \f$ G_m(t) \f$ .
	 */
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

	/**
	 * Computes derivatives.
	 *
	 * @param [in] z: Time.
	 * @param [in] Sve: Current Sve.
	 * @param [out] derivatives: d(Sve)/dz
	 */
	void computeFlowMap(const scalar_t& z, const state_vector_t& Sve, state_vector_t& derivatives) {

		// denormalized time
		scalar_t t = switchingTimeFinal_ - z;

		GvFunc_.interpolate(t, __Gv);
		size_t greatestLessTimeStampIndex = GvFunc_.getGreatestLessTimeStampIndex();
		GmFunc_.interpolate(t, __Gm, greatestLessTimeStampIndex);

		// Error equation for the equivalent system
		derivatives = __Gm.transpose()*Sve+__Gv;
	}


private:
	size_t activeSubsystem_;
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;

	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > GvFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > GmFunc_;

	// members required in computeFlowMap
	state_vector_t __Gv;
	state_matrix_t __Gm;
};

} // namespace ocs2

#endif /* SEQUENTIALERROREQUATION_H_ */
