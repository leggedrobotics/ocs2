/*
 * ComDynamicsDerivativeBase.h
 *
 *  Created on: Nov 10, 2017
 *      Author: farbod
 */

#ifndef COMDYNAMICSDERIVATIVEBASE_H_
#define COMDYNAMICSDERIVATIVEBASE_H_

#include <array>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

#include <ocs2_core/dynamics/DerivativesBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/misc/SphericalCoordinate.h"
#include "ocs2_robotic_tools/common/AngularVelocityMapping.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class ComDynamicsDerivativeBase : public ocs2::DerivativesBase<12,12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM = 12,
		INPUT_DIM = 12
	};

	using Base = ocs2::DerivativesBase<12,12>;

	using com_model_t = ComModelBase<JOINT_COORD_SIZE>;
	using kinematic_model_t = KinematicsModelBase<JOINT_COORD_SIZE>;

	using base_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t;
	using joint_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t;
	using base_jacobian_matrix_t = Eigen::Matrix<double,6,JOINT_COORD_SIZE>;
	using state_joint_matrix_t = Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE>;

	ComDynamicsDerivativeBase(const kinematic_model_t& kinematicModel, const com_model_t& comModel,
			const double& gravitationalAcceleration=9.81, const bool& constrainedIntegration=true)
	: Base(),
	  kinematicModelPtr_(kinematicModel.clone()),
	  comModelPtr_(comModel.clone()),
	  o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  constrainedIntegration_(constrainedIntegration)
	{
		if (gravitationalAcceleration<0)  throw std::runtime_error("Gravitational acceleration should be a positive value.");
	}

	~ComDynamicsDerivativeBase() override {}

	/**
	 * copy constructor
	 */
	ComDynamicsDerivativeBase(const ComDynamicsDerivativeBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  o_gravityVector_(rhs.o_gravityVector_),
	  constrainedIntegration_(rhs.constrainedIntegration_)
	{}

	/**
	 * clone this class.
	 */
	ComDynamicsDerivativeBase<JOINT_COORD_SIZE>* clone() const override;


	/**
	 * Set state and input.
	 */
	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

	/**
	 * set the current state and contact force input
	 */
	void setData(const joint_coordinate_t& qJoints,	const joint_coordinate_t& dqJoints);

	/**
	 * calculate and retrieve the A matrix
	 */
	void getFlowMapDerivativeState(state_matrix_t& A) final override;

	/**
	 * calculate and retrieve the B matrix
	 */
	void getFlowMapDerivativeInput(state_input_matrix_t& B) final override;

	/**
	 * calculate and retrieve the approximate derivatives w.r.t. joints
	 */
	void getApproximateDerivativesJoint(state_joint_matrix_t& partrialF_q);

	/**
	 * calculate and retrieve the approximate derivatives w.r.t. joints' velocities
	 */
	void getApproximateDerivativesJointVelocity(state_joint_matrix_t& partrialF_dq);

	/**
	 * user interface for CoM dynamics: get Base to CoM vector in CoM frame.
	 */
	void getBase2CoMInComFrame(Eigen::Vector3d& com_base2CoM) const;

	/**
	 * user interface for CoM dynamics: get Base Pose.
	 */
	void getBasePose(base_coordinate_t& basePose) const;

	/**
	 * user interface for CoM dynamics: get Base Pose.
	 */
	void getBaseLocalVelocities(base_coordinate_t& baseLocalVelocities) const;

	/**
	 * user interface for CoM dynamics: get CoM to stance feet in CoM frame.
	 */
	void getCom2StanceFeetInComFrame(std::array<Eigen::Vector3d,4>& com_com2StanceFeet) const;

	/**
	 * user interface for CoM dynamics: get Base to stance feet in CoM frame.
	 */
	void getBase2StanceFeetInComFrame(std::array<Eigen::Vector3d,4>& com_base2StanceFeet) const;

	/**
	 * user interface for CoM dynamics: get CoM Jacobian with respect to base.
	 */
	void getComJacobianInBaseFrame(base_jacobian_matrix_t& b_comJacobain) const;

	/**
	 * user interface for CoM dynamics: get feet Jacobian with respect to base.
	 */
	void getFeetJacobiansInBaseFrame(std::array<base_jacobian_matrix_t,4>& b_feetJacobains) const;

	/**
	 * user interface for CoM dynamics: get time derivative of CoM Jacobian with respect to base.
	 */
	void getComJacobianTimeDerivativeInBaseFrame(base_jacobian_matrix_t& b_comJacobainTimeDerivative) const;

	/**
	 * user interface for CoM dynamics: get the rotation matrix from Base frame to Origin frame.
	 */
	void getRotationMatrixBasetoOrigin(Eigen::Matrix3d& o_R_b) const;

private:
	typename kinematic_model_t::Ptr	kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	Eigen::Vector3d o_gravityVector_;

	bool constrainedIntegration_;

	base_coordinate_t  q_base_;
	base_coordinate_t  dq_base_; // local angular and linear velocity of Base
	joint_coordinate_t qJoints_;
	joint_coordinate_t dqJoints_;

	Eigen::Matrix3d o_R_b_;

	// jacobian of the angular velocities to Euler angle derivatives transformation D(d\theta / dt) = [ d/d(theta), d/d(w_omega_b) ]
	Eigen::Matrix<double,3,6> jacobianOfAngularVelocityMapping_;

	Eigen::Vector3d com_base2CoM_;
	base_jacobian_matrix_t b_comJacobain_;
	base_jacobian_matrix_t b_comJacobainTimeDerivative_;
	std::array<Eigen::Vector3d,4> com_base2StanceFeet_;
	std::array<Eigen::Vector3d,4> com_com2StanceFeet_;

	std::array<base_jacobian_matrix_t,4> b_feetJacobains_;

	// Inertia matrix
	Eigen::Matrix<double, 6, 6> M_;
	Eigen::Matrix<double, 6, 6> dMdt_;
	Eigen::Matrix<double, 6, 6> MInverse_;
	std::array<Eigen::Matrix<double,6,6>, 12> partialM_;
};

} // end of namespace switched_model

#include "implementation/ComDynamicsDerivativeBase.h"

#endif /* COMDYNAMICSDERIVATIVEBASE_H_ */
