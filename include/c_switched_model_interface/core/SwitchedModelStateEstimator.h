/*
 * SwitchedModelStateEstimator.h
 *
 *  Created on: Jun 5, 2016
 *      Author: farbod
 */

#ifndef SWITCHEDMODELSTATEESTIMATOR_H_
#define SWITCHEDMODELSTATEESTIMATOR_H_

#include <array>
#include <Eigen/Dense>

#include "c_switched_model_interface/core/ComModelBase.h"

namespace switched_model {

template< size_t JOINT_COORD_SIZE >
class SwitchedModelStateEstimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double,12,1>						com_model_state_t;
	typedef Eigen::Matrix<double,12+JOINT_COORD_SIZE,1>		comkino_model_state_t;
	typedef Eigen::Matrix<double,12+2*JOINT_COORD_SIZE,1>	full_model_state_t;
	typedef Eigen::Matrix<double,12+2*JOINT_COORD_SIZE,1> 	rbd_model_state_t;

	/******************************************************************************************************/
	SwitchedModelStateEstimator(const typename ComModelBase<JOINT_COORD_SIZE>::Ptr& comModelPtr)
	: comModelPtr_(comModelPtr)
	{}

	/******************************************************************************************************/
	~SwitchedModelStateEstimator() {}

	/******************************************************************************************************/
	/**
	 * calculate comkino switched model state from the rbd model state.
	 */
	void estimateComkinoModelState(const rbd_model_state_t& rbdState, comkino_model_state_t& comkinoState) {

		// calculate the CoM state
		com_model_state_t comState;
		estimateComState(rbdState, comState);

		comkinoState << comState,
				rbdState.template segment<JOINT_COORD_SIZE>(6);
	}

	/******************************************************************************************************/
	/**
	 * calculate full switched model state from the rbd model state.
	 */
	void estimateFullModelState(const rbd_model_state_t& rbdState, full_model_state_t& fullState) {

		// calculate the CoM state
		com_model_state_t comState;
		estimateComState(rbdState, comState);

		fullState << comState,
				rbdState.template segment<JOINT_COORD_SIZE>(6),
				rbdState.template tail<JOINT_COORD_SIZE>();
	}

	/******************************************************************************************************/
	/**
	 * calculate CoMDynamics state from the generalized coordinates and their velocities.
	 */
	void estimateComState(const rbd_model_state_t& rbdState, com_model_state_t& comState) {

		// Joints' velocities
		Eigen::VectorBlock<const rbd_model_state_t,12> qJoints = rbdState.template segment<JOINT_COORD_SIZE>(6);
		Eigen::VectorBlock<const rbd_model_state_t,12> dqJoints = rbdState.template tail<JOINT_COORD_SIZE>();
		// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
		Eigen::Matrix3d o_R_b = RotationMatrixBasetoOrigin(rbdState.template head<3>());
		// base to CoM displacement in the CoM frame
		Eigen::Vector3d b_r_com = comModelPtr_->comPositionBaseFrame(qJoints);
		// CoM Jacobin in the Base frame
		Eigen::Matrix<double,6,JOINT_COORD_SIZE> b_comJacobain = comModelPtr_->comJacobainBaseFrame(qJoints);

		// local velocities of Base
		Eigen::Matrix<double,6,1> comLocalVelocities;
		comLocalVelocities.head<3>() = rbdState.template segment<3>(18) + b_comJacobain.template topRows<3>()*dqJoints;
		comLocalVelocities.tail<3>() = rbdState.template segment<3>(21) + b_comJacobain.template bottomRows<3>()*dqJoints
				- b_r_com.cross(rbdState.template segment<3>(18));

		comState << rbdState.template head<3>(),
				rbdState.template segment<3>(3) + o_R_b * b_r_com,
				comLocalVelocities;
	}


private:

	typename ComModelBase<JOINT_COORD_SIZE>::Ptr comModelPtr_;

};

}  // end of switched_model namespace

#endif /* end of include guard: SWITCHEDMODELSTATEESTIMATOR_H_ */
