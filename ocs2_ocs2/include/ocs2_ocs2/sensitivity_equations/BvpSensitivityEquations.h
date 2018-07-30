/*
 * BvpSensitivityEquations.h
 *
 *  Created on: Jul 19, 2018
 *      Author: farbod
 */

#ifndef BVPSENSITIVITYEQUATIONS_OCS2_H_
#define BVPSENSITIVITYEQUATIONS_OCS2_H_

#include <Eigen/StdVector>
#include <vector>
#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/integration/ODE_Base.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * BVP sensitivity equations.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class BvpSensitivityEquations : public ODE_Base<STATE_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ODE_Base<STATE_DIM> BASE;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t       scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
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
	typedef typename DIMENSIONS::state_input_matrix_t 		state_input_matrix_t;
	typedef typename DIMENSIONS::state_input_matrix_array_t state_input_matrix_array_t;
	typedef typename DIMENSIONS::constraint1_vector_t       constraint1_vector_t;
	typedef typename DIMENSIONS::constraint1_vector_array_t constraint1_vector_array_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_t       constraint1_state_matrix_t;
	typedef typename DIMENSIONS::constraint1_state_matrix_array_t constraint1_state_matrix_array_t;
	typedef typename DIMENSIONS::controller_t controller_t;

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
			const input_state_matrix_array_t* CmConstrainedPtr,
			const state_vector_array_t* QvPtr,
			const state_vector_array_t* flowMapPtr,
			const state_vector_array_t* costatePtr,
			const constraint1_vector_array_t* lagrangianPtr,
			const scalar_array_t* SsTimeStampPtr,
			const input_state_matrix_array_t* KmConstrainedPtr,
			const state_matrix_array_t* SmPtr)  {

		BASE::resetNumFunctionCalls();

		switchingTimeStart_ = switchingTimeStart;
		switchingTimeFinal_ = switchingTimeFinal;
		scalingFactor_      = switchingTimeFinal - switchingTimeStart;

		AmFunc_.setTimeStamp(timeStampPtr);
		AmFunc_.setData(AmPtr);
		BmFunc_.setTimeStamp(timeStampPtr);
		BmFunc_.setData(BmPtr);
		CmFunc_.setTimeStamp(timeStampPtr);
		CmFunc_.setData(CmPtr);
		AmConstrainedFunc_.setTimeStamp(timeStampPtr);
		AmConstrainedFunc_.setData(AmConstrainedPtr);
		CmConstrainedFunc_.setTimeStamp(timeStampPtr);
		CmConstrainedFunc_.setData(CmConstrainedPtr);
		QvFunc_.setTimeStamp(timeStampPtr);
		QvFunc_.setData(QvPtr);

		flowMapFunc_.setTimeStamp(timeStampPtr);
		flowMapFunc_.setData(flowMapPtr);
		costateFunc_.setTimeStamp(timeStampPtr);
		costateFunc_.setData(costatePtr);
		lagrangianFunc_.setTimeStamp(timeStampPtr);
		lagrangianFunc_.setData(lagrangianPtr);

		KmConstrainedFunc_.setTimeStamp(SsTimeStampPtr);
		KmConstrainedFunc_.setData(KmConstrainedPtr);
		SmFunc_.setTimeStamp(SsTimeStampPtr);
		SmFunc_.setData(SmPtr);
	}

	/**
	 * Reset the Riccati equation
	 */
	void reset() {

		AmFunc_.reset();
		BmFunc_.reset();
		CmFunc_.reset();
		AmConstrainedFunc_.reset();
		CmConstrainedFunc_.reset();
		QvFunc_.reset();

		flowMapFunc_.reset();
		costateFunc_.reset();
		lagrangianFunc_.reset();

		KmConstrainedFunc_.reset();
		SmFunc_.reset();
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

		AmFunc_.interpolate(t, Am_);
		size_t greatestLessTimeStampIndex = AmFunc_.getGreatestLessTimeStampIndex();
		BmFunc_.interpolate(t, Bm_, greatestLessTimeStampIndex);
		CmFunc_.interpolate(t, Cm_, greatestLessTimeStampIndex);
		AmConstrainedFunc_.interpolate(t, AmConstrained_, greatestLessTimeStampIndex);
		CmConstrainedFunc_.interpolate(t, CmConstrained_, greatestLessTimeStampIndex);
		QvFunc_.interpolate(t, Qv_, greatestLessTimeStampIndex);

		flowMapFunc_.interpolate(t, flowMap_, greatestLessTimeStampIndex);
		costateFunc_.interpolate(t, costate_, greatestLessTimeStampIndex);
		lagrangianFunc_.interpolate(t, lagrangian_, greatestLessTimeStampIndex);

		KmConstrainedFunc_.interpolate(t, KmConstrained_);
		greatestLessTimeStampIndex = KmConstrainedFunc_.getGreatestLessTimeStampIndex();
		SmFunc_.interpolate(t, Sm_, greatestLessTimeStampIndex);

		// here we have used RmConstrained = (I-DmConstrained).transpose() * Rm
		// and Km = -(I-DmConstrained) \tilde{L} - CmConstrained_
		dMvdt_ = (AmConstrained_ + Bm_*(CmConstrained_+KmConstrained_)).transpose()*Mv +
				multiplier_*(Qv_ + Am_.transpose()*costate_ + Cm_.transpose()*lagrangian_ + Sm_*flowMap_);

		// TODO: delete me
//		std::cout << "[" << t << "]" << std::endl;
//		std::cout << "qv" << std::endl;
//		std::cout <<
//				multiplier_*(
//						Qv_ + Am_.transpose()*costate_ + Cm_.transpose()*lagrangian_
//				).transpose()
//				<< std::endl;
//		std::cout << "gv" << std::endl;
//		std::cout << multiplier_*(flowMap_).transpose() << std::endl;
//		std::cout << "S" << std::endl;
//		std::cout << Sm_ << std::endl;
//		std::cout << "qv+S*gv" << std::endl;
//		std::cout <<
//				multiplier_*(
//						Qv_ + Am_.transpose()*costate_ + Cm_.transpose()*lagrangian_ + Sm_*flowMap_
//						).transpose()
//				<< std::endl;
//		std::cout << std::endl;

		dMvdz = scalingFactor_ * dMvdt_;
	}


private:
	scalar_t switchingTimeStart_ = 0.0;
	scalar_t switchingTimeFinal_ = 1.0;
	scalar_t scalingFactor_ = 1.0;

	scalar_t multiplier_ = 0.0;

	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t>> AmFunc_;
	LinearInterpolation<state_input_matrix_t,Eigen::aligned_allocator<state_input_matrix_t>> BmFunc_;
	LinearInterpolation<constraint1_state_matrix_t,Eigen::aligned_allocator<constraint1_state_matrix_t>> CmFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t>> AmConstrainedFunc_;
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t>> CmConstrainedFunc_;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t>> QvFunc_;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t>> flowMapFunc_;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t>> costateFunc_;
	LinearInterpolation<constraint1_vector_t,Eigen::aligned_allocator<constraint1_vector_t>> lagrangianFunc_;
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t>> KmConstrainedFunc_;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t>> SmFunc_;

	state_matrix_t Am_;
	state_input_matrix_t Bm_;
	constraint1_state_matrix_t Cm_;
	state_matrix_t AmConstrained_;
	input_state_matrix_t CmConstrained_;
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
