/*
 * ComKinoConstraintBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */

#ifndef COMKINOCONSTRAINTBASE_H_
#define COMKINOCONSTRAINTBASE_H_

#include <array>
#include <memory>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ocs2_core/constraint/ConstraintBase.h>

#include "c_switched_model_interface/core/SwitchedModel.h"
#include "c_switched_model_interface/core/KinematicsModelBase.h"
#include "c_switched_model_interface/core/ComModelBase.h"
#include "c_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"
#include "c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"
#include <c_switched_model_interface/core/Options.h>

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class ComKinoConstraintBase : public ocs2::ConstraintBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE, SwitchedModelLogicRulesBase<JOINT_COORD_SIZE>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM = 12+JOINT_COORD_SIZE,
		INPUT_DIM = 12+JOINT_COORD_SIZE
	};

	typedef SwitchedModelLogicRulesBase<JOINT_COORD_SIZE> 	logic_rules_t;
	typedef typename logic_rules_t::feet_cpg_ptr_t 			feet_cpg_ptr_t;
	typedef ocs2::LogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_t> logic_rules_machine_t;

	typedef ocs2::ConstraintBase<STATE_DIM, INPUT_DIM, logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename Base::scalar_t scalar_t;
	typedef typename Base::state_matrix_t state_matrix_t;
	typedef typename Base::state_vector_t state_vector_t;
	typedef typename Base::input_vector_t input_vector_t;
	typedef typename Base::control_gain_matrix_t control_gain_matrix_t;
	typedef typename Base::constraint1_vector_t 		constraint1_vector_t;
	typedef typename Base::constraint1_state_matrix_t 	constraint1_state_matrix_t;
	typedef typename Base::constraint1_control_matrix_t constraint1_control_matrix_t;
	typedef typename Base::constraint2_vector_t 		constraint2_vector_t;
	typedef typename Base::constraint2_state_matrix_t 	constraint2_state_matrix_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef Eigen::Matrix<double,6,JOINT_COORD_SIZE> base_jacobian_matrix_t;


	ComKinoConstraintBase(const kinematic_model_t* kinematicModelPtr, const com_model_t* comModelPtr,
			const scalar_t& gravitationalAcceleration=9.81, const Options& options = Options())

	: Base(),
	  kinematicModelPtr_(kinematicModelPtr->clone()),
	  comModelPtr_(comModelPtr->clone()),
	  o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  options_(options)
	{
		if (gravitationalAcceleration<0)
			throw std::runtime_error("Gravitational acceleration should be a positive value.");
	}

	/**
	 * copy construntor of ComKinoConstraintBase
	 */
	ComKinoConstraintBase(const ComKinoConstraintBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  o_gravityVector_(rhs.o_gravityVector_),
	  options_(rhs.options_)
	{}

	virtual ~ComKinoConstraintBase() {}

	/**
	 * clone ComKinoConstraintBase class.
	 */
	virtual ComKinoConstraintBase<JOINT_COORD_SIZE>* clone() const override;

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
	 * Set the current state and contact force input
	 *
	 * @param t: current time
	 * @param x: current switched state vector (centrodial dynamics plus joints' angles)
	 * @param u: current switched input vector (contact forces plus joints' velocities)
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

	/**
	 * Equality constraint type-1 consists of states and inputs.
	 */
	virtual void computeConstriant1(size_t& numConstraint1, constraint1_vector_t& g1) override;

	/**
	 * Equality and inequality constraint type-2 consists of states.
	 */
	virtual void computeConstriant2(size_t& numConstraint2, constraint2_vector_t& g2) override;

	/**
	 * Equality and inequality final constraint type-2 consists of states.
	 */
	virtual void computeFinalConstriant2(size_t& numFinalConstraint2, constraint2_vector_t& g2Final) override;

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


private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	Eigen::Vector3d o_gravityVector_;
	Options options_;

	const logic_rules_t* logicRulesPtr_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	const std::vector<EndEffectorConstraintBase::ConstPtr>* endEffectorStateConstraintsPtr_;

	std::array<bool,4> stanceLegs_;
	std::array<bool,4> nextPhaseStanceLegs_;

	const feet_cpg_ptr_t* zDirectionRefsPtr_;

	joint_coordinate_t qJoints_;
	joint_coordinate_t dqJoints_;
	base_coordinate_t basePose_;
	base_coordinate_t baseLocalVelocities_;

	Eigen::Matrix3d o_R_b_;

	Eigen::Vector3d com_base2CoM_;
	base_jacobian_matrix_t b_comJacobain_;
	base_jacobian_matrix_t b_comJacobainTimeDerivative_;

	std::array<Eigen::Vector3d,4> com_com2StanceFeet_;
	std::array<Eigen::Vector3d,4> com_base2StanceFeet_;
	std::array<Eigen::Vector3d,4> o_origin2StanceFeet_;

	std::array<bool,4> feetConstraintIsActive_;
	std::array<scalar_t,4> feetConstraintValues_;
	std::array<Eigen::Vector3d,4> feetConstraintJacobains_;

	std::array<base_jacobian_matrix_t,4> b_feetJacobains_;
	std::array<base_jacobian_matrix_t,4> b_feetJacobainsTimeDerivative_;

	// Inertia matrix
	Eigen::Matrix<double, 6, 6> M_;
	Eigen::Matrix<double, 6, 6> dMdt_;
	Eigen::Matrix<double, 6, 6> MInverse_;

	std::string algorithmName_;
};

} // end of namespace switched_model

#include "implementation/ComKinoConstraintBase.h"

#endif /* COMKINOCONSTRAINTBASE_H_ */
