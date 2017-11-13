/*
 * ComKinoDynamicsDerivativeBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */

#ifndef COMKINODYNAMICSDERIVATIVEBASE_H_
#define COMKINODYNAMICSDERIVATIVEBASE_H_

#include <array>
#include <memory>
#include <string>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ocs2_core/dynamics/DerivativesBase.h>

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/core/KinematicsModelBase.h"
#include "c_switched_model_interface/core/ComModelBase.h"
#include "ComDynamicsDerivativeBase.h"
#include "misc/FeetZDirectionPlanner.h"
#include "state_constraint/EndEffectorConstraintBase.h"

template <size_t JOINT_COORD_SIZE>
class ComKinoDynamicsDerivativeBase : public ocs2::DerivativesBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ocs2::DerivativesBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE> Base;
	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef Eigen::Matrix<double,6,JOINT_COORD_SIZE> base_jacobian_matrix_t;


	ComKinoDynamicsDerivativeBase(const typename kinematic_model_t::Ptr& kinematicModelPtr, const typename com_model_t::Ptr& comModelPtr,
			const std::array<bool,4>& stanceLegs, const double& gravitationalAcceleration=9.81, const Options& options = Options(),
			const FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner=NULL,
			const std::vector<EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints = std::vector<EndEffectorConstraintBase::Ptr>())

	: kinematicModelPtr_(kinematicModelPtr->clone()),
	  comModelptr_(comModelPtr->clone()),
	  o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  comDynamicsDerivative_(kinematicModelPtr, comModelPtr, gravitationalAcceleration, options.constrainedIntegration_),
	  stanceLegs_(stanceLegs),
	  options_(options),
	  feetZDirectionPlanner_(feetZDirectionPlanner),
	  endEffectorStateConstraints_(endEffectorStateConstraints.size())
	{
		if (gravitationalAcceleration<0)  throw std::runtime_error("Gravitational acceleration should be a positive value.");

		for (size_t i=0; i<endEffectorStateConstraints.size(); i++)
			endEffectorStateConstraints_[i] = endEffectorStateConstraints[i]->clone();
	}

	/**
	 * copy construntor of ComKinoDynamicsDerivativeBase
	 */
	ComKinoDynamicsDerivativeBase(const ComKinoDynamicsDerivativeBase& rhs)

	: kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  o_gravityVector_(rhs.o_gravityVector_),
	  comDynamicsDerivative_(rhs.comDynamicsDerivative_),
	  stanceLegs_(rhs.stanceLegs_),
	  options_(rhs.options_),
	  feetZDirectionPlanner_(rhs.feetZDirectionPlanner_->clone()),
	  endEffectorStateConstraints_(rhs.endEffectorStateConstraints_.size())
	{
		for (size_t i=0; i<rhs.endEffectorStateConstraints_.size(); i++)
			endEffectorStateConstraints_[i] = rhs.endEffectorStateConstraints_[i]->clone();
	}

	virtual ~ComKinoDynamicsDerivativeBase() {}

	/**
	 * clone ComKinoDynamicsDerivativeBase class.
	 */
	virtual std::shared_ptr<Base> clone() const override;

	/**
	 * Initializes the model: This method should always be called at the very first call of the model derivatives.
	 */
	virtual void initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
			const state_vector_t& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL)  override;

	/**
	 * Set the current state and contact force input
	 *
	 * @param t
	 * @param x
	 * @param u
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) override;


	virtual void getDerivativeState(state_matrix_t& A)  override;

	virtual void getDerivativesControl(control_gain_matrix_t& B)  override;

	virtual void getConstraint1DerivativesState(constraint1_state_matrix_t& C)  override;

	virtual void getConstraint1DerivativesControl(constraint1_control_matrix_t& D)  override;

	virtual void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override;

	virtual void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F) override;

	/**
	 * to map local angular velocity \omega_W expressed in body coordinates, to changes in Euler Angles expressed in an inertial frame q_I
	 * we have to map them via \dot{q}_I = H \omega_W, where H is the matrix defined in kindr getMappingFromLocalAngularVelocityToDiff.
	 * You can see the kindr cheat sheet to figure out how to build this matrix. The following code computes the Jacobian of \dot{q}_I
	 * with respect to \q_I and \omega_W. Thus the lower part of the Jacobian is H and the upper part is dH/dq_I \omega_W. We include
	 * both parts for more efficient computation. The following code is computed using auto-diff.
	 * @param eulerAnglesXyz
	 * @param angularVelocity
	 * @return
	 */
	Eigen::Matrix<double,6,3> JacobianOfAngularVelocityMapping(const Eigen::Vector3d& eulerAnglesXyz,
			const Eigen::Vector3d& angularVelocity);

	/*
	 * set the stance legs
	 */
	void setStanceLegs (const std::array<bool,4>& stanceLegs)  {  stanceLegs_ = stanceLegs;  }

	/*
	 * get the model's stance leg
	 */
	void getStanceLegs (std::array<bool,4>& stanceLegs)  {  stanceLegs = stanceLegs_;  }

	/*
	 * set the swing legs z direction CPGs planner
	 */
	void setSwingLegsCpgsPlanner (const FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner) {  feetZDirectionPlanner_ = feetZDirectionPlanner; }

	/*
	 * set the swing legs z direction CPGs
	 */
	void setSwingLegsCpgs (const std::array<CpgBase::Ptr,4>& feetZDirectionCPGs) {  feetZDirectionCPGs_ = feetZDirectionCPGs; }

	/*
	 * set the endEffectorStateConstraints vector pointer
	 */
	void setEndEffectorStateConstraints (const std::vector<EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints) {
		endEffectorStateConstraints_ = endEffectorStateConstraints;
	}


private:

	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	Eigen::Vector3d   o_gravityVector_;

	ComDynamicsBase<JOINT_COORD_SIZE> comDynamics_;

	std::array<bool,4> stanceLegs_;
	std::array<bool,4> nextPhaseStanceLegs_;

	Options options_;

	FeetZDirectionPlannerBase::Ptr feetZDirectionPlanner_;
	CpgBase::PtrArray feetZDirectionCPGs_;

	std::vector<EndEffectorConstraintBase::Ptr> endEffectorStateConstraints_;

	ComDynamicsDerivativeBase<JOINT_COORD_SIZE> comDynamicsDerivative_;

	joint_coordinate_t qJoints_;
	joint_coordinate_t dqJoints_;
	base_coordinate_t basePose_;
	base_coordinate_t baseLocalVelocities_;

	Eigen::Matrix3d o_R_b_;

	Eigen::Vector3d com_base2CoM_;
	base_jacobian_matrix_t b_comJacobain_;
	base_jacobian_matrix_t b_comJacobainTimeDerivative_;
	std::array<Eigen::Vector3d,4> com_com2StanceFeet_;

	std::array<bool,4> feetConstraintIsActive_;
	std::array<Eigen::Vector3d,4> feetConstraintJacobains_;

	std::array<base_jacobian_matrix_t,4> b_feetJacobains_;
	std::array<base_jacobian_matrix_t,4> b_feetJacobainsTimeDerivative_;

	std::string algorithmName_;
};


#include "implementation/ComKinoDynamicsDerivativeBase.h"

#endif /* COMKINODYNAMICSDERIVATIVEBASE_H_ */
