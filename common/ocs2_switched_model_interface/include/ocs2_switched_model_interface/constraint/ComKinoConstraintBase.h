/*
 * ComKinoConstraintBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: Farbod
 */

#ifndef COMKINOCONSTRAINTBASE_H_
#define COMKINOCONSTRAINTBASE_H_

#include <ocs2_core/constraint/ConstraintBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/Model_Settings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE,
		size_t STATE_DIM=12+JOINT_COORD_SIZE,
		size_t INPUT_DIM=12+JOINT_COORD_SIZE>
class ComKinoConstraintBase : public ocs2::ConstraintBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM_ = STATE_DIM,
		INPUT_DIM_ = INPUT_DIM,
		NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS
	};

	typedef SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double> logic_rules_t;
	typedef typename logic_rules_t::foot_cpg_t            foot_cpg_t;
	typedef typename logic_rules_t::feet_cpg_ptr_t        feet_cpg_ptr_t;
	typedef typename logic_rules_t::feet_cpg_const_ptr_t  feet_cpg_const_ptr_t;

	typedef ocs2::ConstraintBase<STATE_DIM, INPUT_DIM> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef std::vector<int>                           int_array_t;
	typedef typename Base::scalar_t                    scalar_t;
	typedef typename Base::scalar_array_t              scalar_array_t;
	typedef typename Base::state_vector_t              state_vector_t;
	typedef typename Base::state_vector_array_t        state_vector_array_t;
	typedef typename Base::input_vector_t              input_vector_t;
	typedef typename Base::input_vector_array_t        input_vector_array_t;
	typedef typename Base::state_matrix_t              state_matrix_t;
	typedef typename Base::state_matrix_array_t        state_matrix_array_t;
	typedef typename Base::input_matrix_t              input_matrix_t;
	typedef typename Base::input_matrix_array_t        input_matrix_array_t;
	typedef typename Base::state_input_matrix_t        state_input_matrix_t;
	typedef typename Base::input_state_matrix_t        input_state_matrix_t;
	typedef typename Base::input_state_matrix_array_t  input_state_matrix_array_t;
	typedef typename Base::constraint1_vector_t        constraint1_vector_t;
	typedef typename Base::constraint1_vector_array_t  constraint1_vector_array_t;
	typedef typename Base::constraint1_state_matrix_t  constraint1_state_matrix_t;
	typedef typename Base::constraint1_input_matrix_t  constraint1_input_matrix_t;
	typedef typename Base::constraint2_vector_t        constraint2_vector_t;
	typedef typename Base::constraint2_vector_array_t  constraint2_vector_array_t;
	typedef typename Base::constraint2_state_matrix_t  constraint2_state_matrix_t;


	typedef Eigen::Matrix<double,6,JOINT_COORD_SIZE>                      base_jacobian_matrix_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t      contact_flag_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t   base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t  joint_coordinate_t;


	ComKinoConstraintBase(
			const kinematic_model_t& kinematicModel,
			const com_model_t& comModel,
			std::shared_ptr<const logic_rules_t> logicRulesPtr,
			const Model_Settings& options = Model_Settings())

	: Base()
	, kinematicModelPtr_(kinematicModel.clone())
	, comModelPtr_(comModel.clone())
	, logicRulesPtr_(std::move(logicRulesPtr))
	, options_(options)
	{
		if (!logicRulesPtr_) {
			throw std::runtime_error("[ComKinoConstraintBase] logicRules cannot be a nullptr");
		}
	}

	/**
	 * copy constructor of ComKinoConstraintBase
	 */
	ComKinoConstraintBase(const ComKinoConstraintBase& rhs)

	: Base(rhs)
	, kinematicModelPtr_(rhs.kinematicModelPtr_->clone())
	, comModelPtr_(rhs.comModelPtr_->clone())
	, logicRulesPtr_(rhs.logicRulesPtr_)
	, options_(rhs.options_)
	{}

	~ComKinoConstraintBase() override = default;

	/**
	 * clone ComKinoConstraintBase class.
	 */
	ComKinoConstraintBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>* clone() const override;
	
	/**
	 * Set the current state and contact force input
	 *
	 * @param t: current time
	 * @param x: current switched state vector (centroidal dynamics plus joints' angles)
	 * @param u: current switched input vector (contact forces plus joints' velocities)
	 */
	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

	/**
	 * Computes the state-input equality constraints.
	 *
	 * @param [out] e: The state-input equality constraints value.
	 */
	void getConstraint1(constraint1_vector_t& e) override;

	/**
	 * Get the number of state-input active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of state-input active equality constraints.
	 */
	size_t numStateInputConstraint(const scalar_t& time) override;

	/**
	 * get the state-only equality constraints.
	 *
	 * @param [out] h: The state-only equality constraints value.
	 */
	void getConstraint2(constraint2_vector_t& h) override;

	/**
	 * Get the number of state-only active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of state-only active equality constraints.
	 */
	size_t numStateOnlyConstraint(const scalar_t& time) override;

	/**
 	* Gets the inequality constraints.
 	*
 	*  h_i(x, u, t) >= 0
 	*
 	* @param [out] h: The inequality constraints value.
 	*/
	void getInequalityConstraint(scalar_array_t& h) override;

	/**
	 * Get the number of inequality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of inequality constraints.
	 */
	size_t numInequalityConstraint(const scalar_t& time) override;

	/**
	 * Compute the final state-only equality constraints.
	 *
	 * @param [out] h_f: The final state-only equality constraints value.
	 */
	void getFinalConstraint2(constraint2_vector_t& h_f) override;

	/**
	 * Get the number of final state-only active equality constraints.
	 *
	 * @param [in] time: time.
	 * @return number of final state-only active equality constraints.
	 */
	size_t numStateOnlyFinalConstraint(const scalar_t& time) override;

	/**
	 * calculate and retrieve the C matrix (i.e. the state derivative of the state-input constraints w.r.t. state vector).
	 * Note that only nc1 top rows are valid where nc1 is the number of active state-input constraints at the current time.
	 *
	 * @param C: a nc1-by-nx matrix
	 */
	void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override;

	/**
	 * calculate and retrieve the D matrix (i.e. the state derivative of the state-input constraints w.r.t. input vector).
	 * Note that only nc1 top rows are valid where nc1 is the number of active state-input constraints at the current time.
	 *
	 * @param D: a nc1-by-nu matrix
	 */
	void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) override;

	/**
	 * calculate and retrieve the the derivative of the state-input constraints w.r.t. event times.
	 * g1DevArray[i] is a vector of dimension MAX_CONSTRAINT1_DIM_ which is the partial derivative of
	 * state-input equality constraints with respect to i'th event time.
	 *
	 * @param [out] g1DevArray: an array of nc1-by-1 vector.
	 */
	void getConstraint1DerivativesEventTimes(constraint1_vector_array_t& g1DevArray) override;

	/**
	 * The dhdx matrix at a given operating point for the linearized inequality constraints,
	 * \f$ 0.5 * \delta x ddhdxdx_i(t) \delta x + \delta u ddhdudx_i(t) \delta x + 0.5 * \delta u ddhdudu_i(t) \delta x
	 *    + dhdx_i(t) \delta x + dhdu_i(t) \delta u + e(t) >= 0 \f$.
	 * @param [out] dhdx: \f$ dhdx(t) \f$ matrix.
	 */
	void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override;

	/**
	 * The dhdu matrix at a given operating point for the linearized inequality constraints,
	 * \f$ 0.5 * \delta x ddhdxdx_i(t) \delta x + \delta u ddhdudx_i(t) \delta x + 0.5 * \delta u ddhdudu_i(t) \delta x
	 *    + dhdx_i(t) \delta x + dhdu_i(t) \delta u + e(t) >= 0 \f$.
	 * @param [out] dhdu: \f$ dhdu(t) \f$ matrix.
	 */
	void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override;

	/**
	 * The ddhdxdx matrices at a given operating point for the linearized inequality constraints,
	 * \f$ 0.5 * \delta x ddhdxdx_i(t) \delta x + \delta u ddhdudx_i(t) \delta x + 0.5 * \delta u ddhdudu_i(t) \delta x
	 *    + dhdx_i(t) \delta x + dhdu_i(t) \delta u + e(t) >= 0 \f$.
	 * @param [out] ddhdxdx: \f$ ddhdxdx(t) \f$ matrix.
	 */
	void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override;

	/**
	 * The ddhdudu matrices at a given operating point for the linearized inequality constraints,
	 * \f$ 0.5 * \delta x ddhdxdx_i(t) \delta x + \delta u ddhdudx_i(t) \delta x + 0.5 * \delta u ddhdudu_i(t) \delta x
	 *    + dhdx_i(t) \delta x + dhdu_i(t) \delta u + e(t) >= 0 \f$.
	 * @param [out] ddhudu: \f$ ddhdudu(t) \f$ matrix.
	 */
	void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override;

	/**
	 * The ddhdudu matrices at a given operating point for the linearized inequality constraints,
	 * \f$ 0.5 * \delta x ddhdxdx_i(t) \delta x + \delta u ddhdudx_i(t) \delta x + 0.5 * \delta u ddhdudu_i(t) \delta x
	 *    + dhdx_i(t) \delta x + dhdu_i(t) \delta u + e(t) >= 0 \f$.
	 * @param [out] ddhudu: \f$ ddhdudu(t) \f$ matrix.
	 */
	void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override;

	/**
	 * calculate and retrieve the F matrix (i.e. the state derivative of the state-only constraints w.r.t. state vector).
	 * Note that only nc2 top rows are valid where nc2 is the number of active state-only constraints at the current time.
	 *
	 * @param F: a nc2-by-nx matrix
	 */
	void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override;

	/**
	 * * calculate and retrieve the F matrix (i.e. the state derivative of the final state-only constraints w.r.t. state vector).
	 * Note that only nc2Final top rows are valid where nc2Final is the number of active final state-only constraints at the current time.
	 *
	 * @param F_final: a nc2Final-by-nx matrix
	 */
	void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_final) override;

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
	Model_Settings options_;

	std::shared_ptr<const logic_rules_t> logicRulesPtr_;

	size_t numEventTimes_;

	size_t         activeSubsystem_;
	contact_flag_t stanceLegs_;
	contact_flag_t nextPhaseStanceLegs_;

	std::array<const foot_cpg_t*, NUM_CONTACT_POINTS_> zDirectionRefsPtr_;

	std::array<int_array_t,NUM_CONTACT_POINTS_> startTimesIndices_;
	std::array<int_array_t,NUM_CONTACT_POINTS_> finalTimesIndices_;

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
};

} // end of namespace switched_model

#include "implementation/ComKinoConstraintBase.h"

#endif /* COMKINOCONSTRAINTBASE_H_ */
