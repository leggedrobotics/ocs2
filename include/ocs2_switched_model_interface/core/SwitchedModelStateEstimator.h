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

#include "ocs2_switched_model_interface/core/ComModelBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class SwitchedModelStateEstimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef double scalar_t;

	typedef Eigen::Matrix<scalar_t, 12, 1>						com_model_state_t;
	typedef Eigen::Matrix<scalar_t, 12+JOINT_COORD_SIZE, 1>		comkino_model_state_t;
	typedef Eigen::Matrix<scalar_t, 12+2*JOINT_COORD_SIZE, 1> 	full_model_state_t;
	typedef Eigen::Matrix<scalar_t, 12+2*JOINT_COORD_SIZE, 1> 	rbd_model_state_t;
	typedef Eigen::Matrix<scalar_t, JOINT_COORD_SIZE, 1> 		joint_coordinate_t;

	/******************************************************************************************************/
	SwitchedModelStateEstimator(const ComModelBase<JOINT_COORD_SIZE>& comModel)
	: comModelPtr_(comModel.clone())
	{}

	/******************************************************************************************************/
	/**
	 * copy constructor
	 */
	SwitchedModelStateEstimator(const SwitchedModelStateEstimator& rhs)
	: comModelPtr_(rhs.comModelPtr_->clone())
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
	 * Calculates CoMDynamics state from the generalized coordinates and their velocities.
	 */
	void estimateComState(const rbd_model_state_t& rbdState, com_model_state_t& comState) {

		// Joints' velocities
                joint_coordinate_t qJoints = rbdState.template segment<JOINT_COORD_SIZE>(6);
		joint_coordinate_t dqJoints = rbdState.template tail<JOINT_COORD_SIZE>();
		// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
		Eigen::Matrix3d o_R_b = RotationMatrixBasetoOrigin(rbdState.template head<3>());
		// base to CoM displacement in the CoM frame
		Eigen::Vector3d b_r_com = comModelPtr_->comPositionBaseFrame(qJoints);
		// CoM Jacobin in the Base frame
		Eigen::Matrix<scalar_t,6,JOINT_COORD_SIZE> b_comJacobain = comModelPtr_->comJacobainBaseFrame(qJoints);

		// local velocities of Base
		Eigen::Matrix<scalar_t,6,1> comLocalVelocities;
		comLocalVelocities.head<3>() = rbdState.template segment<3>(18) + b_comJacobain.template topRows<3>()*dqJoints;
		comLocalVelocities.tail<3>() = rbdState.template segment<3>(21) + b_comJacobain.template bottomRows<3>()*dqJoints
				- b_r_com.cross(rbdState.template segment<3>(18));

		comState << rbdState.template head<3>(),
				rbdState.template segment<3>(3) + o_R_b * b_r_com,
				comLocalVelocities;
	}

	/******************************************************************************************************/
	/**
	 * Calculates the RBD model state from the comkino switched model state and joint velocities.
	 *
	 * @param [in] comkinoState: comkino switched model state.
	 * @param [in] dqJoints: joint velocities.
	 * @param [in] rbdState: RBD model state
	 */
	void estimateRbdModelState(
			const comkino_model_state_t& comkinoState,
			const joint_coordinate_t& dqJoints,
			rbd_model_state_t& rbdState) {

		typedef Eigen::Matrix<scalar_t, 6, 1> base_coordinate_t;

                base_coordinate_t comPose = comkinoState.template segment<6>(0);
                base_coordinate_t comLocalVelocities = comkinoState.template segment<6>(6);
		joint_coordinate_t qJoints = comkinoState.template segment<JOINT_COORD_SIZE>(12);

		base_coordinate_t basePose;
		comModelPtr_->calculateBasePose(qJoints, comPose, basePose);

		base_coordinate_t baseLocalVelocities;
		comModelPtr_->calculateBaseLocalVelocities(qJoints, dqJoints, comLocalVelocities, baseLocalVelocities);

		rbdState <<
				basePose,
				qJoints,
				baseLocalVelocities,
				dqJoints;
	}

private:

	typename ComModelBase<JOINT_COORD_SIZE>::Ptr comModelPtr_;

};

}  // end of switched_model namespace

#endif /* end of include guard: SWITCHEDMODELSTATEESTIMATOR_H_ */
