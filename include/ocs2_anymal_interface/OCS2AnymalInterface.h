/*
 * OCS2AnymalInterface.h
 *
 *  Created on: Sep 4, 2016
 *      Author: farbod
 */

#ifndef OCS2ANYMALINTERFACE_H_
#define OCS2ANYMALINTERFACE_H_

#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamics.h>
#include <ocs2_anymal_switched_model/dynamics_derivative/AnymalComKinoDynamicsDerivative.h>
#include <ocs2_anymal_switched_model/constraint/AnymalComKinoConstraint.h>
#include <ocs2_anymal_switched_model/cost/AnymalCost.h>
#include <ocs2_anymal_switched_model/initialization/AnymalComKinoOperatingPoints.h>
#include <ocs2_anymal_switched_model/misc/AnymalWeightCompensationForces.h>

#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>

namespace anymal {

class OCS2AnymalInterface : public switched_model::OCS2QuadrupedInterface<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<OCS2AnymalInterface> Ptr;

	typedef switched_model::OCS2QuadrupedInterface<12> BASE;

	typedef AnymalComKinoDynamics           	system_dynamics_t;
	typedef AnymalComKinoDynamicsDerivative 	system_dynamics_derivative_t;
	typedef AnymalComKinoConstraint     		constraint_t;
	typedef AnymalCost		             		cost_funtion_t;
	typedef AnymalComKinoOperatingPoints	 	operating_point_t;
	using mpc_slq_t =   ocs2::MPC_SLQ<BASE::state_dim_, BASE::input_dim_, BASE::logic_rules_t>;

	OCS2AnymalInterface(const std::string& pathToConfigFolder);

	virtual ~OCS2AnymalInterface() = default;

	/**
	 * setup all optimizes
	 */
	void setupOptimizer(
			const logic_rules_ptr_t& logicRulesPtr,
			const mode_sequence_template_t* modeSequenceTemplatePtr,
			slq_base_ptr_t& slqPtr,
			mpc_ptr_t& mpcPtr) override;

	/**
	 * Designs weight compensating input.
	 *
	 * @param [in] switchedState: Switched model state.
	 * @param [out] uForWeightCompensation: Weight compensating input.
	 */
	void designWeightCompensatingInput(
			const state_vector_t& switchedState,
			input_vector_t& uForWeightCompensation) override;

  	controlled_system_base_ptr_t getSystemDynamicsPtr() override {
	  	return controlled_system_base_ptr_t(new system_dynamics_t(modelSettings_));
  	}

        system_dynamics_t const * getDynamicsPtr() override {return dynamicsPtr_.get(); }
        system_dynamics_derivative_t const * getDynamicsDerivativesPtr() override {return dynamicsDerivativesPtr_.get(); }

        cost_funtion_t* getCostPtr() override {return costFunctionPtr_.get(); }

protected:
	// dynamics
	std::unique_ptr<system_dynamics_t> 				dynamicsPtr_;
	// dynamics derivatives
	std::unique_ptr<system_dynamics_derivative_t> 	dynamicsDerivativesPtr_;
	// constraints
	std::unique_ptr<constraint_t> 					constraintsPtr_;
	// cost function
	std::unique_ptr<cost_funtion_t> 				costFunctionPtr_;
	// operating points
	std::unique_ptr<operating_point_t> 				operatingPointsPtr_;

	AnymalWeightCompensationForces weightCompensationForces_;
};

} // end of namespace anymal

#endif /* OCS2ANYMALINTERFACE_H_ */
