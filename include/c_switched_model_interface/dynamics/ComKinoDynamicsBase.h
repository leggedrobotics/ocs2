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

#include <c_switched_model_interface/core/SwitchedModel.h>
#include <c_switched_model_interface/core/KinematicsModelBase.h>
#include <c_switched_model_interface/core/ComModelBase.h>
#include <c_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>
#include <c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h>
#include <c_switched_model_interface/core/Options.h>
#include "ComDynamicsBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class ComKinoDynamicsBase : public ocs2::ControlledSystemBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE, SwitchedModelLogicRulesBase<JOINT_COORD_SIZE>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM = 12+JOINT_COORD_SIZE,
		INPUT_DIM = 12+JOINT_COORD_SIZE
	};

	typedef SwitchedModelLogicRulesBase<JOINT_COORD_SIZE> logic_rules_t;
	typedef ocs2::LogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_t> logic_rules_machine_t;

	typedef ocs2::ControlledSystemBase<STATE_DIM, INPUT_DIM, logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename Base::scalar_t scalar_t;
	typedef typename Base::state_vector_t state_vector_t;
	typedef typename Base::input_vector_t input_vector_t;
	typedef typename Base::constraint1_vector_t constraint1_vector_t;
	typedef typename Base::constraint2_vector_t constraint2_vector_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;

	ComKinoDynamicsBase(const kinematic_model_t* kinematicModelPtr, const com_model_t* comModelPtr,
			const double& gravitationalAcceleration=9.81, const Options& options = Options())

	: Base(),
	  kinematicModelPtr_(kinematicModelPtr->clone()),
	  comModelPtr_(comModelPtr->clone()),
	  o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  options_(options),
	  comDynamics_(kinematicModelPtr, comModelPtr, gravitationalAcceleration, options.constrainedIntegration_)
	{
		if (gravitationalAcceleration < 0)
			throw std::runtime_error("Gravitational acceleration should be a positive value (e.g. +9.81).");
	}

	/**
	 * copy construntor
	 */
	ComKinoDynamicsBase(const ComKinoDynamicsBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  o_gravityVector_(rhs.o_gravityVector_),
	  options_(rhs.options_),
	  comDynamics_(rhs.comDynamics_)
	{}

	virtual ~ComKinoDynamicsBase() {}

	/**
	 * clone ComKinoDynamicsBase class.
	 */
	virtual ComKinoDynamicsBase<JOINT_COORD_SIZE>* clone() const  override;

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
	 * Calculates the extended state time evolution based on the current extended state and control input.
	 *
	 * The extended state (x) consists of:
	 * 		+ CoM position and orientation (6-states)
	 * 		+ CoM linear and angular velocities (6-states)
	 * 		+ feet's XY position in Origin Frame (8-states)
	 * 		+ hyQ's joints (12-joints)
	 *
	 * The control input (u) consists of:
	 * 		+ feet's contact forces (12-inputs = 3-XYZ-forces * 4-feet)
	 * 		+ feet XY linear momentum in the Frame which coincides with Base and is parallel to Origin (8-inputs = 2-XY * 4-feet)
	 *
	 * The extended state time derivatives (dxdt) consists of:
	 * 		+ CoM linear and angular velocities (6-states)
	 * 		+ CoM linear and angular accelerations (6-states)
	 * 		+ feet's XY velocities (8-states)
	 * 		+ hyQ's joints' angel time derivatives (12-state)
	 *
	 */
	virtual void computeDerivative(const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u,
			state_vector_t& dxdt) override;

	/**
	 * set the stance legs
	 */
	void setStanceLegs (const std::array<bool,4>& stanceLegs);

	/**
	 * get the stance legs
	 */
	void getStanceLegs (std::array<bool,4>& stanceLegs) const;


private:

	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	Eigen::Vector3d o_gravityVector_;
	Options options_;

	ComDynamicsBase<JOINT_COORD_SIZE> comDynamics_;

	const logic_rules_t* logicRulesPtr_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	std::array<bool,4> stanceLegs_;

	Eigen::Vector3d com_base2CoM_;
	Eigen::Matrix<double,6,12> b_comJacobain_;
	std::array<Eigen::Vector3d,4> com_base2StanceFeet_;

	std::string algorithmName_;
};

} // end of namespace switched_model

#include "implementation/ComKinoDynamicsBase.h"

#endif /* COMKINODYNAMICSBASE_H_ */
