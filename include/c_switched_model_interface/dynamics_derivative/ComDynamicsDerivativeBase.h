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

#include "c_switched_model_interface/misc/SphericalCoordinate.h"

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/core/KinematicsModelBase.h"
#include "c_switched_model_interface/core/ComModelBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class ComDynamicsDerivativeBase : public ocs2::DerivativesBase<12,12,ocs2::NullLogicRules<12,12>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM = 12,
		INPUT_DIM = 12
	};

	typedef ocs2::NullLogicRules<12,12> logic_rules_t;
	typedef ocs2::LogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_t> logic_rules_machine_t;

	typedef ocs2::DerivativesBase<12,12,logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef Eigen::Matrix<double,6,JOINT_COORD_SIZE> base_jacobian_matrix_t;
	typedef Eigen::Matrix<double,JOINT_COORD_SIZE,JOINT_COORD_SIZE> state_joint_matrix_t;

	ComDynamicsDerivativeBase(const kinematic_model_t* kinematicModelPtr, const com_model_t* comModelPtr,
			const double& gravitationalAcceleration=9.81, const bool& constrainedIntegration=true)

	: Base(),
	  kinematicModelPtr_(kinematicModelPtr->clone()),
	  comModelPtr_(comModelPtr->clone()),
	  o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  constrainedIntegration_(constrainedIntegration)
	{
		if (gravitationalAcceleration<0)  throw std::runtime_error("Gravitational acceleration should be a positive value.");
	}

	virtual ~ComDynamicsDerivativeBase() {}

	/**
	 * copy construntor
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
	virtual ComDynamicsDerivativeBase<JOINT_COORD_SIZE>* clone() const override;

	/**
	 * Initializes the system dynamics. This method should always be called at the very first call of the model.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(const logic_rules_machine_t& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL) override;

	/**
	 * Set state and input.
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

	/**
	 * set the current state and contact force input
	 */
	void setData(const std::array<bool,4>& stanceLegs,
			const joint_coordinate_t& qJoints,
			const joint_coordinate_t& dqJoints);

	/**
	 * calculate and retrieve the A matrix
	 */
	virtual void getDerivativeState(state_matrix_t& A)  override;

	/**
	 * calculate and retrieve the B matrix
	 */
	virtual void getDerivativesControl(control_gain_matrix_t& B)  override;

	/**
	 * calculate and retrieve the approximate derivatives w.r.t. joints
	 */
	void getApproximateDerivativesJoint(state_joint_matrix_t& partrialF_q);

	/**
	 * calculate and retrieve the approximate derivatives w.r.t. joints' velocities
	 */
	void getApproximateDerivativesJointVelocity(state_joint_matrix_t& partrialF_dq);

	/**
	 * to map local angular velocity \omega_W expressed in body coordinates, to changes in Euler Angles expressed in an inertial frame q_I
	 * we have to map them via \dot{q}_I = H \omega_W, where H is the matrix defined in kindr getMappingFromLocalAngularVelocityToDiff.
	 * You can see the kindr cheat sheet to figure out how to build this matrix. The following code computes the Jacobian of \dot{q}_I
	 * with respect to \q_I and \omega_W. Thus the lower part of the Jacobian is H and the upper part is dH/dq_I \omega_W. We include
	 * both parts for more efficient computation. The following code is computed using auto-diff.
	 *
	 * @param eulerAnglesXyz
	 * @param angularVelocity
	 * @return
	 */
	static Eigen::Matrix<double,6,3> JacobianOfAngularVelocityMapping(const Eigen::Vector3d& eulerAnglesXyz,
			const Eigen::Vector3d& angularVelocity);

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

	std::array<bool,4> stanceLegs_;

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

	std::string algorithmName_;
};

} // end of namespace switched_model

#include "implementation/ComDynamicsDerivativeBase.h"

#endif /* COMDYNAMICSDERIVATIVEBASE_H_ */
