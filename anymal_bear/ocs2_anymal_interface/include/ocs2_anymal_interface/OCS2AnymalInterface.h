/*
 * OCS2AnymalInterface.h
 *
 *  Created on: Sep 4, 2016
 *      Author: farbod
 */

#pragma once

#include <ocs2_anymal_switched_model/constraint/AnymalComKinoConstraintAd.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>
#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>

namespace anymal {

class OCS2AnymalInterface final : public switched_model::OCS2QuadrupedInterface<12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<OCS2AnymalInterface>;

  using BASE = switched_model::OCS2QuadrupedInterface<12>;

  using system_dynamics_t            = switched_model::ComKinoSystemDynamicsAd;
  using system_dynamics_derivative_t = switched_model::ComKinoSystemDynamicsAd;
  using constraint_t                 = switched_model::AnymalComKinoConstraintAd;
  using cost_function_t              = switched_model::SwitchedModelCostBase;
  using operating_point_t            = switched_model::ComKinoOperatingPointsBase;

  explicit OCS2AnymalInterface(const std::string& pathToConfigFolder);

  ~OCS2AnymalInterface() override = default;

  void setupOptimizer(const logic_rules_ptr_t& logicRulesPtr, const mode_sequence_template_t* modeSequenceTemplatePtr,
                      slq_base_ptr_t& slqPtr, mpc_ptr_t& mpcPtr) override;

  const system_dynamics_t& getDynamics() const override { return *dynamicsPtr_; }

  const system_dynamics_derivative_t& getDynamicsDerivatives() const override { return *dynamicsDerivativesPtr_; }

  const cost_function_t& getCost() const override { return *costFunctionPtr_; }

  const constraint_t* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const rollout_base_t& getRollout() const override { return *timeTriggeredRolloutPtr_; }

 private:

  std::unique_ptr<system_dynamics_t> dynamicsPtr_;
  std::unique_ptr<system_dynamics_derivative_t> dynamicsDerivativesPtr_;
  std::unique_ptr<constraint_t> constraintsPtr_;
  std::unique_ptr<cost_function_t> costFunctionPtr_;
  std::unique_ptr<operating_point_t> operatingPointsPtr_;
  std::unique_ptr<rollout_base_t> timeTriggeredRolloutPtr_;
};

}  // end of namespace anymal
