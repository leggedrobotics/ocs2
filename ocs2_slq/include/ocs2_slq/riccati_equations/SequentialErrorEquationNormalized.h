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
#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * This class implements the time-normalized Error equation of Riccati equation.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SequentialErrorEquationNormalized : public ODE_Base<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ODE_Base<STATE_DIM> BASE;

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
	SequentialErrorEquationNormalized(const bool& useMakePSD)
	: useMakePSD_(useMakePSD)
	{}

	/**
	 * Default destructor.
	 */
	~SequentialErrorEquationNormalized() = default;

	/**
	 * Sets coefficients of the model.
	 *
	 * @param [in] switchingTimeStart: The start time of the subsystem.
	 * @param [in] switchingTimeFinal: The final time of the subsystem.
	 * @param [in] timeStampPtr: A pointer to the time stamp trajectory.
	 * @param [in] GvPtr: A pointer to the trajectory of \f$ G_v(t) \f$ .
	 * @param [in] GmPtr: A pointer to the trajectory of \f$ G_m(t) \f$ .
	 */
	void setData(const scalar_t& switchingTimeStart, const scalar_t& switchingTimeFinal,
			scalar_array_t* const timeStampPtr, state_vector_array_t* const GvPtr, state_matrix_array_t* const GmPtr)  {

		BASE::resetNumFunctionCalls();

		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;

		GvFunc_.setTimeStamp(timeStampPtr);
		GvFunc_.setData(GvPtr);
		GmFunc_.setTimeStamp(timeStampPtr);
		GmFunc_.setData(GmPtr);
	}

	/**
	 * Reset the Error Riccati equation
	 */
	void reset() {
		GvFunc_.reset();
		GmFunc_.reset();
	}

	/**
	 * Error Riccati jump map at switching moments
	 *
	 * @param [in] time: Normalized transition time
	 * @param [in] state: transition state
	 * @param [out] mappedState: mapped state after transition
	 */
	void computeJumpMap(const scalar_t& z, const state_vector_t& state,
			state_vector_t& mappedState) override {

		mappedState = state;
	}

	/**
	 * Computes derivatives.
	 *
	 * @param [in] z: Normalized time.
	 * @param [in] Sve: Current Sve.
	 * @param [out] derivatives: d(Sve)/dz
	 */
	void computeFlowMap(const scalar_t& z, const state_vector_t& Sve, state_vector_t& derivatives) {

		BASE::numFunctionCalls_++;

		// denormalized time
		const scalar_t t = switchingTimeFinal_ + (switchingTimeStart_-switchingTimeFinal_)*z;

		GvFunc_.interpolate(t, Gv_);
		size_t greatestLessTimeStampIndex = GvFunc_.getGreatestLessTimeStampIndex();
		GmFunc_.interpolate(t, Gm_, greatestLessTimeStampIndex);

		// Error equation for the equivalent system
		derivatives = (switchingTimeFinal_-switchingTimeStart_)*(Gm_.transpose()*Sve+Gv_);
	}


private:
	bool useMakePSD_;
	scalar_t switchingTimeStart_;
	scalar_t switchingTimeFinal_;

	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > GvFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > GmFunc_;

	// members required in computeFlowMap
	state_vector_t Gv_;
	state_matrix_t Gm_;
};

} // namespace ocs2

#endif /* SEQUENTIALERROREQUATIONNORMALIZED_OCS2_H_ */
