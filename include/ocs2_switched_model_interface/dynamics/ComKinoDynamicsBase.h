/*
 * ComKinoDynamicsBase.h
 *
 *  Created on: Nov 10, 2017
 *      Author: farbod
 */

#ifndef COMKINODYNAMICSBASE_H_
#define COMKINODYNAMICSBASE_H_

#include <array>
#include <memory>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ocs2_core/dynamics/ControlledSystemBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/Model_Settings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/dynamics/ComDynamicsBase.h"
#include "ocs2_switched_model_interface/ground/FlatGroundProfile.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"
#include "ocs2_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE,
    size_t STATE_DIM=12+JOINT_COORD_SIZE,
    size_t INPUT_DIM=12+JOINT_COORD_SIZE,
    class LOGIC_RULES_T=SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double>>
class ComKinoDynamicsBase : public
ocs2::ControlledSystemBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE, LOGIC_RULES_T>
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
	typedef ocs2::LogicRulesMachine<logic_rules_t> logic_rules_machine_t;

	typedef ocs2::ControlledSystemBase<STATE_DIM, INPUT_DIM, logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename Base::scalar_t scalar_t;
	typedef typename Base::state_vector_t state_vector_t;
	typedef typename Base::input_vector_t input_vector_t;
	typedef typename Base::dynamic_vector_t dynamic_vector_t;
	typedef typename Base::constraint1_vector_t constraint1_vector_t;
	typedef typename Base::constraint2_vector_t constraint2_vector_t;

	typedef Eigen::Matrix<scalar_t,3,1> vector3d_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t     contact_flag_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;

	typedef GroundProfileBase<scalar_t> 		ground_profile_t;
	typedef typename ground_profile_t::Ptr 		ground_profile_ptr_t;
	typedef FlatGroundProfile<scalar_t> 		flat_ground_profile_t;
	typedef typename flat_ground_profile_t::Ptr flat_ground_profile_ptr_t;

	ComKinoDynamicsBase(const kinematic_model_t& kinematicModel, const com_model_t& comModel,
			const Model_Settings& options = Model_Settings(),
			const ground_profile_ptr_t& groundProfilePtr = flat_ground_profile_ptr_t(new flat_ground_profile_t()))
	: Base(),
	  kinematicModelPtr_(kinematicModel.clone()),
	  comModelPtr_(comModel.clone()),
	  o_gravityVector_(0.0, 0.0, -options.gravitationalAcceleration_),
	  options_(options),
	  comDynamics_(kinematicModel, comModel,
								 options.gravitationalAcceleration_,
								 options.constrainedIntegration_,
								 options.enforceFrictionConeConstraint_,
								 options.frictionCoefficient_),
	  groundProfilePtr_(groundProfilePtr)
	{}

	/**
	 * copy constructor
	 */
	ComKinoDynamicsBase(const ComKinoDynamicsBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  o_gravityVector_(rhs.o_gravityVector_),
	  options_(rhs.options_),
	  comDynamics_(rhs.comDynamics_),
	  groundProfilePtr_(rhs.groundProfilePtr_)
	{}

	virtual ~ComKinoDynamicsBase() = default;

	/**
	 * clone ComKinoDynamicsBase class.
	 */
	virtual ComKinoDynamicsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, LOGIC_RULES_T>* clone() const  override;

	/**
	 * Initializes the system dynamics. This method should always be called at the very first call of the model.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(logic_rules_machine_t& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL) override;

	/**
	 * Calculates the extended state time evolution based on the current extended state and control input.
	 *
	 * The extended state (x) consists of:
	 * 		+ CoM position and orientation (6-states)
	 * 		+ CoM linear and angular velocities (6-states)
	 * 		+ feet's XY position in Origin Frame (8-states)
	 * 		+ hyQ's joints (12-joints)
	 *
	 * The control input (u) consists of:
	 * 		+ feet's contact forces (12-inputs = 3-XYZ-forces * NUM_CONTACT_POINTS_-feet)
	 * 		+ feet XY linear momentum in the Frame which coincides with Base and is parallel
	 * 		  to Origin (8-inputs = 2-XY * NUM_CONTACT_POINTS_-feet)
	 *
	 * The extended state time derivatives (dxdt) consists of:
	 * 		+ CoM linear and angular velocities (6-states)
	 * 		+ CoM linear and angular accelerations (6-states)
	 * 		+ feet's XY velocities (8-states)
	 * 		+ hyQ's joints' angel time derivatives (12-state)
	 *
	 */
	virtual void computeFlowMap(const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u,
			state_vector_t& dxdt) final;

	/**
	 * set the stance legs
	 */
	void setStanceLegs (const contact_flag_t& stanceLegs);

	/**
	 * get the stance legs
	 */
	void getStanceLegs (contact_flag_t& stanceLegs) const;

	/**
	 * State map at the transition time
	 *
	 * @param [in] time: transition time
	 * @param [in] state: transition state
	 * @param [out] mappedState: mapped state after transition
	 */
	virtual void computeJumpMap(
			const scalar_t& time,
			const state_vector_t& state,
			state_vector_t& mappedState) final {

		mappedState = state;
	}

	/**
	 *
	 * @param [in] t: transition time
	 * @param [in] x: transition state
	 * @param [out] guardSurfacesValue: An array of guard surfaces values
	 */
	virtual void computeGuardSurfaces(
			const scalar_t& t,
			const state_vector_t& x,
			dynamic_vector_t& guardSurfacesValue) final;


private:

	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	vector3d_t o_gravityVector_;
	Model_Settings options_;

	ComDynamicsBase<JOINT_COORD_SIZE> comDynamics_;

	logic_rules_t* logicRulesPtr_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	contact_flag_t stanceLegs_;

	vector3d_t com_base2CoM_;
	Eigen::Matrix<scalar_t,6,12> b_comJacobain_;
	std::array<vector3d_t,NUM_CONTACT_POINTS_> com_base2StanceFeet_;

	std::string algorithmName_;

	ground_profile_ptr_t groundProfilePtr_;
};

} // end of namespace switched_model

#include "implementation/ComKinoDynamicsBase.h"

#endif /* COMKINODYNAMICSBASE_H_ */
