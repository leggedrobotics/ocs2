/*
 * ComKinoConstraintBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: Farbod
 */

#ifndef COMKINOCONSTRAINTBASE_H_
#define COMKINOCONSTRAINTBASE_H_

#include <array>
#include <memory>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ocs2_core/constraint/ConstraintBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/Model_Settings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"
#include "ocs2_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE,
		size_t STATE_DIM=12+JOINT_COORD_SIZE,
		size_t INPUT_DIM=12+JOINT_COORD_SIZE,
		class LOGIC_RULES_T=SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double>>
class ComKinoConstraintBase : public
ocs2::ConstraintBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE, LOGIC_RULES_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM_ = STATE_DIM,
		INPUT_DIM_ = INPUT_DIM,
		NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS
	};

	typedef LOGIC_RULES_T logic_rules_t;
	typedef typename logic_rules_t::foot_cpg_t 				foot_cpg_t;
	typedef typename logic_rules_t::feet_cpg_ptr_t 			feet_cpg_ptr_t;
	typedef typename logic_rules_t::feet_cpg_const_ptr_t	feet_cpg_const_ptr_t;
	typedef ocs2::LogicRulesMachine<logic_rules_t> logic_rules_machine_t;

	typedef ocs2::ConstraintBase<STATE_DIM, INPUT_DIM, logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename Base::scalar_t scalar_t;
	typedef typename Base::state_matrix_t state_matrix_t;
	typedef typename Base::state_vector_t state_vector_t;
	typedef typename Base::input_vector_t input_vector_t;
	typedef typename Base::state_input_matrix_t 		state_input_matrix_t;
	typedef typename Base::constraint1_vector_t 		constraint1_vector_t;
	typedef typename Base::constraint1_state_matrix_t 	constraint1_state_matrix_t;
	typedef typename Base::constraint1_input_matrix_t constraint1_input_matrix_t;
	typedef typename Base::constraint2_vector_t 		constraint2_vector_t;
	typedef typename Base::constraint2_state_matrix_t 	constraint2_state_matrix_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t     contact_flag_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef Eigen::Matrix<double,6,JOINT_COORD_SIZE> base_jacobian_matrix_t;


	ComKinoConstraintBase(
			const kinematic_model_t& kinematicModel,
			const com_model_t& comModel,
			const Model_Settings& options = Model_Settings())

	: Base()
	, kinematicModelPtr_(kinematicModel.clone())
	, comModelPtr_(comModel.clone())
	, o_gravityVector_(0.0, 0.0, -options.gravitationalAcceleration_)
	, options_(options)
	{}

	/**
	 * copy constructor of ComKinoConstraintBase
	 */
	ComKinoConstraintBase(const ComKinoConstraintBase& rhs)

	: Base(rhs)
	, kinematicModelPtr_(rhs.kinematicModelPtr_->clone())
	, comModelPtr_(rhs.comModelPtr_->clone())
	, o_gravityVector_(rhs.o_gravityVector_)
	, options_(rhs.options_)
	{}

	virtual ~ComKinoConstraintBase() = default;

	/**
	 * clone ComKinoConstraintBase class.
	 */
	virtual ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const override;

	/**
	 * Initializes the system dynamics. This method should always be called at the very first call of the model.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(
			logic_rules_machine_t& logicRulesMachine,
			const size_t& partitionIndex,
			const char* algorithmName=NULL) override;

	/**
	 * Set the current state and contact force input
	 *
	 * @param t: current time
	 * @param x: current switched state vector (centroidal dynamics plus joints' angles)
	 * @param u: current switched input vector (contact forces plus joints' velocities)
	 */
	virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

	/**
	 * Computes the state-input equality constraints.
	 *
	 * @param [out] e: The state-input equality constraints value.
	 */
	virtual void getConstraint1(constraint1_vector_t& e) override;

	/**
	 * Get the number of state-input active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of state-input active equality constraints.
	 */
	virtual size_t numStateInputConstraint(const scalar_t& time) override;

	/**
	 * get the state-only equality constraints.
	 *
	 * @param [out] h: The state-only equality constraints value.
	 */
	virtual void getConstraint2(constraint2_vector_t& h) override;

	/**
	 * Get the number of state-only active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of state-only active equality constraints.
	 */
	virtual size_t numStateOnlyConstraint(const scalar_t& time) override;

	/**
	 * Compute the final state-only equality constraints.
	 *
	 * @param [out] h_f: The final state-only equality constraints value.
	 */
	virtual void getFinalConstraint2(constraint2_vector_t& h_f) override;

	/**
	 * Get the number of final state-only active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of final state-only active equality constraints.
	 */
	virtual size_t numStateOnlyFinalConstraint(const scalar_t& time) override;

	/**
	 * calculate and retrieve the C matrix (i.e. the state derivative of the state-input constraints w.r.t. state vector).
	 * Note that only nc1 top rows are valid where nc1 is the number of active state-input constraints at the current time.
	 *
	 * @param C: a nc1-by-nx matrix
	 */
	virtual void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override;

	/**
	 * calculate and retrieve the D matrix (i.e. the state derivative of the state-input constraints w.r.t. input vector).
	 * Note that only nc1 top rows are valid where nc1 is the number of active state-input constraints at the current time.
	 *
	 * @param D: a nc1-by-nu matrix
	 */
	virtual void getConstraint1DerivativesControl(constraint1_input_matrix_t& D)  override;

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
	void setStanceLegs (const contact_flag_t& stanceLegs);

	/**
	 * get the model's stance leg
	 */
	void getStanceLegs (contact_flag_t& stanceLegs);


private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	Eigen::Vector3d o_gravityVector_;
	Model_Settings options_;

	logic_rules_t* logicRulesPtr_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	size_t numSubsystems_;

	const std::vector<EndEffectorConstraintBase::ConstPtr>* endEffectorStateConstraintsPtr_;

	contact_flag_t stanceLegs_;
	contact_flag_t nextPhaseStanceLegs_;

	std::array<const foot_cpg_t*, NUM_CONTACT_POINTS_> zDirectionRefsPtr_;

	joint_coordinate_t qJoints_;
	joint_coordinate_t dqJoints_;
	base_coordinate_t basePose_;
	base_coordinate_t baseLocalVelocities_;

	Eigen::Matrix3d o_R_b_;

	Eigen::Vector3d com_base2CoM_;
	base_jacobian_matrix_t b_comJacobain_;
	base_jacobian_matrix_t b_comJacobainTimeDerivative_;

	std::array<Eigen::Vector3d,NUM_CONTACT_POINTS_> com_com2StanceFeet_;
	std::array<Eigen::Vector3d,NUM_CONTACT_POINTS_> com_base2StanceFeet_;
	std::array<Eigen::Vector3d,NUM_CONTACT_POINTS_> o_origin2StanceFeet_;

	std::array<bool,NUM_CONTACT_POINTS_> feetConstraintIsActive_;
	std::array<scalar_t,NUM_CONTACT_POINTS_> feetConstraintValues_;
	std::array<Eigen::Vector3d,NUM_CONTACT_POINTS_> feetConstraintJacobains_;

	std::array<base_jacobian_matrix_t,NUM_CONTACT_POINTS_> b_feetJacobains_;
	std::array<base_jacobian_matrix_t,NUM_CONTACT_POINTS_> b_feetJacobainsTimeDerivative_;

	// Inertia matrix
	Eigen::Matrix<double, 6, 6> M_;
	Eigen::Matrix<double, 6, 6> dMdt_;
	Eigen::Matrix<double, 6, 6> MInverse_;

	std::string algorithmName_;
};

} // end of namespace switched_model

#include "implementation/ComKinoConstraintBase.h"

#endif /* COMKINOCONSTRAINTBASE_H_ */
