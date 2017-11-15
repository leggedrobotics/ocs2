/*
 * SwitchedModelStateEstimator.h
 *
 *  Created on: Jun 5, 2016
 *      Author: farbod
 */

#ifndef HYQ_SWITCHEDMODELSTATEESTIMATOR_H_
#define HYQ_SWITCHEDMODELSTATEESTIMATOR_H_

#include <array>
#include <Eigen/Dense>

#include <iit/robots/hyq/jsim.h>
#include <iit/robots/hyq/inertia_properties.h>

#include "kinematics/SwitchedModelKinematics.h"

#define SIMPLE_MODEL

namespace switched_model {

class SwitchedModelStateEstimator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum {
		COM_STATE_SIZE = 12,
		JOINT_COORDINATE_SIZE = 12,
		SWITCHED_MODEL_STATE_SIZE   = 12+12,
		Full_MODEL_STATE_SIZE       = 36,
		GENERALIZED_COORDINATE_SIZE = 18,
	};

	typedef std::array<Eigen::Vector3d, 4> vector3d_array_t;
	typedef Eigen::Matrix<double,COM_STATE_SIZE,1> com_state_t;
	typedef Eigen::Matrix<double,2*GENERALIZED_COORDINATE_SIZE,1> hyq_state_t;
	typedef Eigen::Matrix<double,GENERALIZED_COORDINATE_SIZE,1> hyq_coordinate_t;
	typedef Eigen::Matrix<double,SWITCHED_MODEL_STATE_SIZE,1> 	switched_model_state_t;
	typedef Eigen::Matrix<double,Full_MODEL_STATE_SIZE,1> 		full_model_state_t;

	SwitchedModelStateEstimator()

		: jsim_(inertiaProperties_, forceTransforms_)
	{}

	~SwitchedModelStateEstimator() {}

	/******************************************************************************************************/
	/*
	 * calculate SwitchedModelDynamics output from the HyQ generalized coordinates and their velocities.
	 */
	static void EstimateOutput(const double& time, const hyq_state_t& hyqState, switched_model_state_t& switchedHyqOutput)  {

		// estimate SwitchedHyQDynamics state
		switched_model_state_t switchedHyqState;
		EstimateSwitchedModelState(time, hyqState, switchedHyqState);

		switchedHyqOutput = switchedHyqState;
	}


	/******************************************************************************************************/
	/*
	 * calculate SwitchedModelDynamics state from the HyQ generalized coordinates and their velocities.
	 */
	static void EstimateSwitchedModelState(const double& time, const hyq_state_t& hyqState, switched_model_state_t& switchedHyqState) {

		// calculate the CoM state
		com_state_t comState;
		EstimateComState(time, hyqState, comState);

		// switchedHyq state
		switchedHyqState << comState, hyqState.segment<JOINT_COORDINATE_SIZE>(6);
	}


	/******************************************************************************************************/
	/*
	 * calculate SwitchedModelDynamics state from the HyQ generalized coordinates and their velocities.
	 */
	static void EstimateFullModelState(const double& time, const hyq_state_t& hyqState, full_model_state_t& fullHyqState) {

		// calculate the CoM state
		com_state_t comState;
		EstimateComState(time, hyqState, comState);

		// switchedHyq state
		fullHyqState << comState, hyqState.segment<JOINT_COORDINATE_SIZE>(6), hyqState.tail<JOINT_COORDINATE_SIZE>();
	}


	/******************************************************************************************************/
	/*
	 * calculate CoMDynamics state from the HyQ generalized coordinates and their velocities.
	 */
	static void EstimateComState(const double& time, const hyq_state_t& hyqState, com_state_t& comState) {

#ifdef SIMPLE_MODEL
		comState << hyqState.head<6>(), hyqState.segment<6>(18);

#else
		// joints' angles
		Eigen::Matrix<double,JOINT_COORDINATE_SIZE,1> qJoints = hyqState.segment<JOINT_COORDINATE_SIZE>(6);

		// rotation matrix
		Eigen::Matrix3d b_R_o = SwitchedModelKinematics::RotationMatrixOrigintoBase(hyqState.head<3>());

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
		Eigen::Vector3d o_r_com = b_R_o.transpose()*b_r_com + hyqState.segment<3>(3);

		// CoM velocity in CoM frame (== World frame)
		Eigen::Matrix<double,6,1> com_comLocalVelocity  = w_comJacobian * hyqState.tail<GENERALIZED_COORDINATE_SIZE>();

		// CoM state
		comState << hyqState.head<3>(), o_r_com, com_comLocalVelocity;
#endif
	}


	/******************************************************************************************************/
	/*
	 * calculate the feet subsystem jacobian
	 */
	void feetSubsystemInertia(const hyq_state_t& hyqState,
			std::array<Eigen::Matrix3d, 4>& wo_feetXYZIneriaTensorInverse,
			std::array<Eigen::Matrix2d, 4>& wo_feetXYIneriaTensorInverse)  {

		// HyQ inverse inertia tensor in the world frame
		Eigen::Matrix<double, 18, 18> hyqIneriaTensor = jsim_.update(hyqState.segment<JOINT_COORDINATE_SIZE>(6));
		Eigen::Matrix<double, 18, 18> hyqIneriaTensorInverse = hyqIneriaTensor.inverse();

		// update Kinematics
		hyqForwardKinematics_.update(hyqState.head<GENERALIZED_COORDINATE_SIZE>());

		// rotation matrix
		Eigen::Matrix3d o_R_b = hyqForwardKinematics_.rotationMatrixOrigintoBase().transpose();

		Eigen::Vector3d b_r_foot;
		Eigen::Matrix<double,6,12> b_J_foot;
		Eigen::Matrix<double,6,18> wo_J_foot;
		for (size_t i=0; i<4; i++)  {
			// foot position in Base frame
			hyqForwardKinematics_.footPositionBaseFrame(i, b_r_foot);
			// foot jacobian in Base frame
			hyqForwardKinematics_.footJacobainBaseFrame(i, b_J_foot);
			// foot subsystem XYZ inverse inertia matrix
			SwitchedModelKinematics::FromBaseJacobianToInertiaJacobian(o_R_b, b_r_foot, b_J_foot, wo_J_foot);
			wo_feetXYZIneriaTensorInverse[i] = wo_J_foot.bottomRows<3>() * hyqIneriaTensorInverse *
					wo_J_foot.bottomRows<3>().transpose();
			// foot subsystem XY inverse inertia matrix
			wo_feetXYIneriaTensorInverse[i] = wo_feetXYZIneriaTensorInverse[i].block<2,2>(0,0);
		}
	}


private:
	SwitchedModelKinematics hyqForwardKinematics_;

	iit::HyQ::dyn::InertiaProperties inertiaProperties_;
	iit::HyQ::ForceTransforms forceTransforms_;
	iit::HyQ::dyn::JSIM jsim_;

};

}  // end of switched_model namespace

#endif /* HYQ_SWITCHEDMODELSTATEESTIMATOR_H_ */
