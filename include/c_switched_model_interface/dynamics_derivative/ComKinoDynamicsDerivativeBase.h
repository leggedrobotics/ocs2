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
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ocs2_core/dynamics/DerivativesBase.h>

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/core/KinematicsModelBase.h"
#include "c_switched_model_interface/core/ComModelBase.h"
#include "ComDynamicsDerivativeBase.h"
#include "c_switched_model_interface/misc/FeetZDirectionPlanner.h"
#include "c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"
#include <c_switched_model_interface/core/Options.h>

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class ComKinoDynamicsDerivativeBase : public ocs2::DerivativesBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ocs2::DerivativesBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE> Base;
	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;
	typedef typename Base::scalar_t scalar_t;
	typedef typename Base::state_matrix_t state_matrix_t;
	typedef typename Base::state_vector_t state_vector_t;
	typedef typename Base::control_vector_t control_vector_t;
	typedef typename Base::control_gain_matrix_t control_gain_matrix_t;
	typedef typename Base::constraint1_state_matrix_t constraint1_state_matrix_t;
	typedef typename Base::constraint1_control_matrix_t constraint1_control_matrix_t;
	typedef typename Base::constraint2_state_matrix_t constraint2_state_matrix_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef Eigen::Matrix<double,6,JOINT_COORD_SIZE> base_jacobian_matrix_t;


	ComKinoDynamicsDerivativeBase(kinematic_model_t* kinematicModelPtr, com_model_t* comModelPtr,
			const std::array<bool,4>& stanceLegs, const double& gravitationalAcceleration=9.81, const Options& options = Options(),
			const FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner=NULL,
			const std::vector<EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints = std::vector<EndEffectorConstraintBase::Ptr>())

	: kinematicModelPtr_(kinematicModelPtr),
	  comModelPtr_(comModelPtr),
	  o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  comDynamicsDerivative_(kinematicModelPtr->clone(), comModelPtr->clone(), gravitationalAcceleration, options.constrainedIntegration_),
	  stanceLegs_(stanceLegs),
	  options_(options),
	  feetZDirectionPlanner_(feetZDirectionPlanner!=nullptr ? feetZDirectionPlanner->clone() : nullptr),
	  endEffectorStateConstraints_(endEffectorStateConstraints.size())
	{
		if (gravitationalAcceleration<0)  throw std::runtime_error("Gravitational acceleration should be a positive value.");

		for (size_t i=0; i<endEffectorStateConstraints.size(); i++){
			if (endEffectorStateConstraints[i] == nullptr)
				throw std::runtime_error("The endEffectorStateConstraints array is not properly initialized.");
			endEffectorStateConstraints_[i] = endEffectorStateConstraints[i]->clone();
		}
	}

	/**
	 * copy construntor of ComKinoDynamicsDerivativeBase
	 */
	ComKinoDynamicsDerivativeBase(const ComKinoDynamicsDerivativeBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  o_gravityVector_(rhs.o_gravityVector_),
	  comDynamicsDerivative_(rhs.comDynamicsDerivative_),
	  stanceLegs_(rhs.stanceLegs_),
	  options_(rhs.options_),
	  feetZDirectionPlanner_(rhs.feetZDirectionPlanner_!=nullptr ? rhs.feetZDirectionPlanner_->clone() : nullptr),
	  endEffectorStateConstraints_(rhs.endEffectorStateConstraints_.size())
	{
		for (size_t i=0; i<rhs.endEffectorStateConstraints_.size(); i++) {
			if (rhs.endEffectorStateConstraints_[i]==nullptr)
				throw std::runtime_error("The endEffectorStateConstraints array is not properly initialized.");
			endEffectorStateConstraints_[i] = rhs.endEffectorStateConstraints_[i]->clone();
		}
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
	 * @param t: current time
	 * @param x: current switched state vector (centrodial dynamics plus joints' angles)
	 * @param u: current switched input vector (contact forces plus joints' velocities)
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const control_vector_t& u) override;

	/**
	 * calculate and retrieve the A matrix (i.e. the state derivative of the dynamics w.r.t. state vector).
	 *
	 * @param A: a nx-by-nx matrix
	 */
	virtual void getDerivativeState(state_matrix_t& A)  override;

	/**
	 * calculate and retrieve the B matrix (i.e. the state derivative of the dynamics w.r.t. input vector).
	 *
	 * @param B: a ny-by-nu matrix
	 */
	virtual void getDerivativesControl(control_gain_matrix_t& B)  override;

	/**
	 * calculate and retrieve the C matrix (i.e. the state derivative of the state-input constraints w.r.t. state vector).
	 * Note that only nc1 top rows are valid where nc1 is the number of active state-input constraints at the current time.
	 *
	 * @param C: a nc1-by-nx matrix
	 */
	virtual void getConstraint1DerivativesState(constraint1_state_matrix_t& C)  override;

	/**
	 * calculate and retrieve the D matrix (i.e. the state derivative of the state-input constraints w.r.t. input vector).
	 * Note that only nc1 top rows are valid where nc1 is the number of active state-input constraints at the current time.
	 *
	 * @param D: a nc1-by-nu matrix
	 */
	virtual void getConstraint1DerivativesControl(constraint1_control_matrix_t& D)  override;

	/**
	 * calculate and retrieve the F matrix (i.e. the state derivative of the state-only constraints w.r.t. state vector).
	 * Note that only nc2 top rows are valid where nc2 is the number of active state-only constraints at the current time.
	 *
	 * @param F: a nc2-by-nx matrix
	 */
	virtual void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override;

	/**
	 * * calculate and retrieve the F matrix (i.e. the state derivative of the final state-only constraints w.r.t. state vector).
	 * Note that only nc2Final top rows are valid where nc2Final is the number of active final state-only constraints at the current time.
	 *
	 * @param F_final: a nc2Final-by-nx matrix
	 */
	virtual void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_final) override;

	/**
	 * set the stance legs
	 */
	void setStanceLegs (const std::array<bool,4>& stanceLegs);

	/**
	 * get the model's stance leg
	 */
	void getStanceLegs (std::array<bool,4>& stanceLegs);

	/**
	 * set the swing legs z direction CPGs plannerinput
	 */
	void setSwingLegsCpgsPlanner (const FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner);

	/**
	 * set the swing legs z direction CPGs
	 */
	void setSwingLegsCpgs (const std::array<CpgBase::Ptr,4>& feetZDirectionCPGs);

	/**
	 * set the endEffectorStateConstraints vector pointer
	 */
	void setEndEffectorStateConstraints (const std::vector<EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints);


private:

	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	Eigen::Vector3d   o_gravityVector_;

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

} // end of namespace switched_model

#include "implementation/ComKinoDynamicsDerivativeBase.h"

#endif /* COMKINODYNAMICSDERIVATIVEBASE_H_ */
