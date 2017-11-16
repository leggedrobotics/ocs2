/*
 * SwitchedModelStateEstimator.h
 *
 *  Created on: Jun 5, 2016
 *      Author: farbod
 */

#ifndef ANYMAL_SWITCHEDMODELSTATEESTIMATOR_H_
#define ANYMAL_SWITCHEDMODELSTATEESTIMATOR_H_


#include <array>
#include <Eigen/Dense>

#include "kinematics/AnymalKinematics.h"
#include "dynamics/AnymalCom.h"

#define SIMPLE_MODEL

namespace anymal {

class SwitchedModelStateEstimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		COM_STATE_SIZE = 12,
		JOINT_COORDINATE_SIZE = 12,
		SWITCHED_MODEL_STATE_SIZE   = 12+12,
		FULL_MODEL_STATE_SIZE       = 36,
		GENERALIZED_COORDINATE_SIZE = 18,
	};

	typedef std::array<Eigen::Vector3d, 4> vector3d_array_t;
	typedef Eigen::Matrix<double,COM_STATE_SIZE,1> com_state_t;
	typedef Eigen::Matrix<double,2*GENERALIZED_COORDINATE_SIZE,1> state_t;
	typedef Eigen::Matrix<double,GENERALIZED_COORDINATE_SIZE,1> coordinate_t;
	typedef Eigen::Matrix<double,SWITCHED_MODEL_STATE_SIZE,1> 	switched_model_state_t;
	typedef Eigen::Matrix<double,FULL_MODEL_STATE_SIZE,1> 		full_model_state_t;

	/******************************************************************************************************/
	/*
	 * calculate SwitchedModelDynamics output from the generalized coordinates and their velocities.
	 */
	static void EstimateOutput(const double& time, const state_t& state, switched_model_state_t& switchedOutput)  {

		// estimate SwitchedDynamics state
		switched_model_state_t switchedState;
		EstimateSwitchedModelState(time, state, switchedState);

		switchedOutput = switchedState;
	}


	/******************************************************************************************************/
	/*
	 * calculate SwitchedModelDynamics state from the generalized coordinates and their velocities.
	 */
	static void EstimateSwitchedModelState(const double& time, const state_t& state, switched_model_state_t& switchedState) {

		// calculate the CoM state
		com_state_t comState;
		EstimateComState(time, state, comState);

		// switched state
		switchedState << comState, state.segment<JOINT_COORDINATE_SIZE>(6);
	}


	/******************************************************************************************************/
	/*
	 * calculate SwitchedModelDynamics state from the generalized coordinates and their velocities.
	 */
	static void EstimateFullModelState(const double& time, const state_t& state, full_model_state_t& fullState) {

		// calculate the CoM state
		com_state_t comState;
		EstimateComState(time, state, comState);

		// switchedHyq state
		fullState << comState, state.segment<JOINT_COORDINATE_SIZE>(6), state.tail<JOINT_COORDINATE_SIZE>();
	}


	/******************************************************************************************************/
	/*
	 * calculate CoMDynamics state from the generalized coordinates and their velocities.
	 */
	static void EstimateComState(const double& time, const state_t& state, com_state_t& comState) {

#ifdef SIMPLE_MODEL
		comState << state.head<6>(), state.segment<6>(18);

#else
		// joints' angles
		Eigen::Matrix<double,JOINT_COORDINATE_SIZE,1> qJoints = state.segment<JOINT_COORDINATE_SIZE>(6);

		// rotation matrix
		Eigen::Matrix3d b_R_o = SwitchedModelKinematics::RotationMatrixOrigintoBase(state.head<3>());

		// CoM position in the Base frame
		Eigen::Vector3d b_r_com;
		SwitchedModelKinematics::ComPositionBaseFrame(qJoints, b_r_com);
		// CoM jacobian in the Base frame
		Eigen::Matrix<double,6,12> b_comJacobian;
		SwitchedModelKinematics::ComJacobainBaseFrame(qJoints, b_comJacobian);
		// CoM jacobian in the World frame
		Eigen::Matrix<double,6,18> w_comJacobian;
		SwitchedModelKinematics::FromBaseJacobianToInertiaJacobian(Eigen::Matrix3d::Identity(), b_r_com, b_comJacobian, w_comJacobian);

		// CoM position in base frame and origin
		Eigen::Vector3d o_r_com = b_R_o.transpose()*b_r_com + state.segment<3>(3);

		// CoM velocity in CoM frame (== World frame)
		Eigen::Matrix<double,6,1> com_comLocalVelocity  = w_comJacobian * state.tail<GENERALIZED_COORDINATE_SIZE>();

		// CoM state
		comState << state.head<3>(), o_r_com, com_comLocalVelocity;
#endif
	}


	/******************************************************************************************************/
	/*
	 * calculate the feet subsystem jacobian
	 */
	void feetSubsystemInertia(const state_t& state,
			std::array<Eigen::Matrix3d, 4>& wo_feetXYZIneriaTensorInverse,
			std::array<Eigen::Matrix2d, 4>& wo_feetXYIneriaTensorInverse)  {

		// inverse inertia tensor in the world frame
		Eigen::Matrix<double, 18, 18> inertiaTensorInverse = anymalCom_.getJsimInverse(state.segment<JOINT_COORDINATE_SIZE>(6));

		// update Kinematics
		anymalKinematics_.update(state.head<GENERALIZED_COORDINATE_SIZE>());

		// rotation matrix
		Eigen::Matrix3d o_R_b = anymalKinematics_.rotationMatrixOrigintoBase().transpose();

		Eigen::Vector3d b_r_foot;
		Eigen::Matrix<double,6,12> b_J_foot;
		Eigen::Matrix<double,6,18> wo_J_foot;
		for (size_t i=0; i<4; i++)  {
			// foot position in Base frame
			anymalKinematics_.footPositionBaseFrame(i, b_r_foot);
			// foot jacobian in Base frame
			anymalKinematics_.footJacobainBaseFrame(i, b_J_foot);
			// foot subsystem XYZ inverse inertia matrix
			AnymalKinematics::FromBaseJacobianToInertiaJacobian(o_R_b, b_r_foot, b_J_foot, wo_J_foot);
			wo_feetXYZIneriaTensorInverse[i] = wo_J_foot.bottomRows<3>() * inertiaTensorInverse *
					wo_J_foot.bottomRows<3>().transpose();
			// foot subsystem XY inverse inertia matrix
			wo_feetXYIneriaTensorInverse[i] = wo_feetXYZIneriaTensorInverse[i].block<2,2>(0,0);
		}
	}


private:

	AnymalCom anymalCom_;
	AnymalKinematics anymalKinematics_;

};

}  // end of anymal namespace

#endif /* end of include guard: ANYMAL_SWITCHEDMODELSTATEESTIMATOR_H_ */
