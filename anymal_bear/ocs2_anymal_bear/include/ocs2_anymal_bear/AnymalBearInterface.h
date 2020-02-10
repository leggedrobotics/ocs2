/*
 * AnymalBearInterface.h
 *
 *  Created on: Sep 4, 2016
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>
#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

#include <ocs2_quadruped_interface/OCS2QuadrupedInterface.h>

namespace anymal {

class AnymalBearInterface final : public switched_model::OCS2QuadrupedInterface<12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = switched_model::OCS2QuadrupedInterface<12>;
  using BASE::mpc_t;

  using system_dynamics_t = switched_model::ComKinoSystemDynamicsAd;
  using system_dynamics_derivative_t = switched_model::ComKinoSystemDynamicsAd;
  using constraint_t = switched_model::ComKinoConstraintBaseAd;
  using cost_function_t = switched_model::SwitchedModelCostBase;
  using operating_point_t = switched_model::ComKinoOperatingPointsBase;

  explicit AnymalBearInterface(const std::string& pathToConfigFolder);

  ~AnymalBearInterface() override = default;

  std::unique_ptr<mpc_t> getMpc() const override;

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
