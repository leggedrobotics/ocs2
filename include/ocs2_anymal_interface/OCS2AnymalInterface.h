/*
 * OCS2AnymalInterface.h
 *
 *  Created on: Sep 4, 2016
 *      Author: farbod
 */

#pragma once

#include <ocs2_anymal_switched_model/constraint/AnymalComKinoConstraint.h>
#include <ocs2_anymal_switched_model/cost/AnymalCost.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamics.h>
#include <ocs2_anymal_switched_model/dynamics_derivative/AnymalComKinoDynamicsDerivative.h>
#include <ocs2_anymal_switched_model/initialization/AnymalComKinoOperatingPoints.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>
#include <ocs2_anymal_switched_model/misc/AnymalWeightCompensationForces.h>

#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>

namespace anymal {

class OCS2AnymalInterface final : public switched_model::OCS2QuadrupedInterface<12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<OCS2AnymalInterface>;

  using BASE = switched_model::OCS2QuadrupedInterface<12>;

  using system_dynamics_t = AnymalComKinoDynamics;
  using system_dynamics_derivative_t = AnymalComKinoDynamicsDerivative;
  using constraint_t = AnymalComKinoConstraint;
  using cost_funtion_t = AnymalCost;
  using operating_point_t = AnymalComKinoOperatingPoints;

  explicit OCS2AnymalInterface(const std::string& pathToConfigFolder);

  ~OCS2AnymalInterface() override = default;

  void setupOptimizer(const logic_rules_ptr_t& logicRulesPtr, const mode_sequence_template_t* modeSequenceTemplatePtr,
                      slq_base_ptr_t& slqPtr, mpc_ptr_t& mpcPtr) override;

  void designWeightCompensatingInput(const state_vector_t& switchedState, input_vector_t& uForWeightCompensation) override;

  const system_dynamics_t& getDynamics() const override { return *dynamicsPtr_; }

  const system_dynamics_derivative_t& getDynamicsDerivatives() const override { return *dynamicsDerivativesPtr_; }

  const cost_funtion_t& getCost() const override { return *costFunctionPtr_; }

 protected:
  // dynamics
  std::unique_ptr<system_dynamics_t> dynamicsPtr_;
  // dynamics derivatives
  std::unique_ptr<system_dynamics_derivative_t> dynamicsDerivativesPtr_;
  // constraints
  std::unique_ptr<constraint_t> constraintsPtr_;
  // cost function
  std::unique_ptr<cost_funtion_t> costFunctionPtr_;
  // operating points
  std::unique_ptr<operating_point_t> operatingPointsPtr_;

  AnymalWeightCompensationForces weightCompensationForces_;
};

}  // end of namespace anymal
