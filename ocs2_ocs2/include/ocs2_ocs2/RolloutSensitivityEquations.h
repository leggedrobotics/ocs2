/**
 * \file RolloutSensitivityEquations.h
 *
 *  Created on: Jan 9, 2016
 *      Author: farbod
 */

#ifndef ROLLOUTSENSITIVITYEQUATIONS_OCS2_H_
#define ROLLOUTSENSITIVITYEQUATIONS_OCS2_H_

#include <array>
#include <Eigen/Dense>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2{

/**
 * Rollout Sensitivity Equations Class
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RolloutSensitivityEquations : public SystemBase<Eigen::Dynamic>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::template LinearFunction_t<INPUT_DIM, Eigen::Dynamic> sensitivity_controller_t;
	typedef typename DIMENSIONS::scalar_t 		scalar_t;
	typedef typename DIMENSIONS::scalar_array_t scalar_array_t;
	typedef typename DIMENSIONS::state_vector_t 	  state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::control_vector_t 		control_vector_t;
	typedef typename DIMENSIONS::control_vector_array_t control_vector_array_t;
	typedef typename DIMENSIONS::control_feedback_t 	  control_feedback_t;
	typedef typename DIMENSIONS::state_matrix_t 	  state_matrix_t;
	typedef typename DIMENSIONS::state_matrix_array_t state_matrix_array_t;
	typedef typename DIMENSIONS::control_gain_matrix_t 		 control_gain_matrix_t;
	typedef typename DIMENSIONS::control_gain_matrix_array_t control_gain_matrix_array_t;

	typedef Eigen::Matrix<double,STATE_DIM,Eigen::Dynamic> 	nabla_state_matrix_t;
	typedef std::vector<nabla_state_matrix_t> 				nabla_state_matrix_array_t;
	typedef Eigen::Matrix<double,INPUT_DIM,Eigen::Dynamic> 	nabla_input_matrix_t;
	typedef std::vector<nabla_input_matrix_t> 				nabla_input_matrix_array_t;
	typedef Eigen::Matrix<double,1,Eigen::Dynamic> 			nabla_scalar_rowvector_t;
	typedef std::vector<nabla_scalar_rowvector_t> 			nabla_scalar_rowvector_array_t;

	RolloutSensitivityEquations()  {}
	~RolloutSensitivityEquations() {}

	/**
	 * Converts to vector
	 * @param [in] numSubsystems
	 * @param [in] nabla_Xm
	 * @param nabla_Xv
	 */
	static void convert2Vector(const size_t& numSubsystems, const nabla_state_matrix_t& nabla_Xm, Eigen::VectorXd& nabla_Xv)  {

		nabla_Xv = Eigen::Map<const Eigen::VectorXd>(nabla_Xm.data(), (numSubsystems-1)*STATE_DIM);
	}

	/**
	 * Converts to Matrix
	 * @param [in] numSubsystems
	 * @param [in] nabla_Xv
	 * @param nabla_Xm
	 */
	static void convert2Matrix(const size_t& numSubsystems, const Eigen::VectorXd& nabla_Xv, nabla_state_matrix_t& nabla_Xm)  {

		nabla_Xm = Eigen::Map<const nabla_state_matrix_t>(nabla_Xv.data(), STATE_DIM, numSubsystems-1);
	}

	/**
	 * Sets Data
	 * @param [in] activeSubsystem
	 * @param [in] switchingTimes
	 * @param [in] sensitivityControllerPtr
	 * @param [in] timeTrajectoryPtr
	 * @param [in] outputTimeDerivativeTrajectoryPtr
	 * @param [in] AmTrajectoryPtr
	 * @param [in] BmTrajectoryPtr
	 */
	void setData(const size_t& activeSubsystem, const scalar_array_t& switchingTimes, const sensitivity_controller_t* sensitivityControllerPtr,
			const scalar_array_t* timeTrajectoryPtr, const state_vector_array_t*  outputTimeDerivativeTrajectoryPtr,
			const state_matrix_array_t* AmTrajectoryPtr, const control_gain_matrix_array_t* BmTrajectoryPtr)  {

		activeSubsystem_ = activeSubsystem;
		switchingTimes_ = switchingTimes;
		numSubsystems_ = switchingTimes_.size()-1;

		KmFunc_.setTimeStamp(&(sensitivityControllerPtr->time_));
		KmFunc_.setData(&(sensitivityControllerPtr->k_));

		LvFunc_.setTimeStamp(&(sensitivityControllerPtr->time_));
		LvFunc_.setData(&(sensitivityControllerPtr->uff_));

		stateTimeDevFunc_.setTimeStamp(timeTrajectoryPtr);
		stateTimeDevFunc_.setData(outputTimeDerivativeTrajectoryPtr);

		AmFunc_.setTimeStamp(timeTrajectoryPtr);
		AmFunc_.setData(AmTrajectoryPtr);
		BmFunc_.setTimeStamp(timeTrajectoryPtr);
		BmFunc_.setData(BmTrajectoryPtr);
	}

	/**
	 * Computes Derivative
	 * @param [in] z
	 * @param [in] nabla_Xv
	 * @param [out] derivatives
	 */
	void computeDerivative(const scalar_t& z, const Eigen::VectorXd& nabla_Xv, Eigen::VectorXd& derivatives) {

		// denormalized time
		scalar_t t = switchingTimes_[activeSubsystem_] + z*(switchingTimes_[activeSubsystem_+1]-switchingTimes_[activeSubsystem_]);

		nabla_state_matrix_t nabla_Xm;
		convert2Matrix(numSubsystems_, nabla_Xv, nabla_Xm);

		state_matrix_t Am;
		AmFunc_.interpolate(t, Am);
		size_t greatestLessTimeStampIndex = AmFunc_.getGreatestLessTimeStampIndex();
		control_gain_matrix_t Bm;
		BmFunc_.interpolate(t, Bm, greatestLessTimeStampIndex);
		state_vector_t dxdt;
		stateTimeDevFunc_.interpolate(t, dxdt, greatestLessTimeStampIndex);

		// compute input sensitivity
		nabla_input_matrix_t nabla_Um;
		computeInputSensitivity(t, nabla_Xm, nabla_Um);

		nabla_state_matrix_t nabla_dXmdz;
		nabla_dXmdz = (switchingTimes_[activeSubsystem_+1]-switchingTimes_[activeSubsystem_])*(Am*nabla_Xm+Bm*nabla_Um);

		for (size_t j=0; j<numSubsystems_-1; j++)  {
			if (j==activeSubsystem_)
				nabla_dXmdz.col(j) += dxdt;
			if (j==activeSubsystem_-1)
				nabla_dXmdz.col(j) -= dxdt;
		}

		convert2Vector(numSubsystems_, nabla_dXmdz, derivatives);
	}

	/**
	 * Computes input sensitivity
	 * @param [in] t
	 * @param [in] nabla_Xm
	 * @param [out] nabla_Um
	 */
	void computeInputSensitivity(const scalar_t& t, const nabla_state_matrix_t& nabla_Xm, nabla_input_matrix_t& nabla_Um) {

		control_feedback_t Km;
		KmFunc_.interpolate(t, Km);
		size_t greatestLessTimeStampIndex = KmFunc_.getGreatestLessTimeStampIndex();

		nabla_input_matrix_t Lv;
		LvFunc_.interpolate(t, Lv, greatestLessTimeStampIndex);

		nabla_Um = Km*nabla_Xm + Lv;
	}


private:
	size_t activeSubsystem_;
	scalar_array_t switchingTimes_;
	size_t numSubsystems_;

	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > KmFunc_;
	LinearInterpolation<nabla_input_matrix_t, Eigen::aligned_allocator<nabla_input_matrix_t> > LvFunc_;

	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > stateTimeDevFunc_;

	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > AmFunc_;
	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc_;

};

} // namespace ocs2

#endif /* ROLLOUTSENSITIVITYEQUATIONS_OCS2_H_ */

