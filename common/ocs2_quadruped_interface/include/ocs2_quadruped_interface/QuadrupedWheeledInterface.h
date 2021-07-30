//
// Created by rgrandia on 19.03.20.
//

#pragma once

#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_core/cost/QuadraticCostFunction.h>

#include <ocs2_switched_model_interface/constraint/AnymalWheelsComKinoConstraintAd.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>
#include <ocs2_switched_model_interface/initialization/ComKinoInitializer.h>

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

namespace switched_model {

class QuadrupedWheeledInterface : public QuadrupedInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using system_dynamics_t = switched_model::ComKinoSystemDynamicsAd;
  using system_dynamics_derivative_t = switched_model::ComKinoSystemDynamicsAd;
  using constraint_t = switched_model::AnymalWheelsComKinoConstraintAd;
  using cost_function_t = switched_model::SwitchedModelCostBase;
  using initializer_t = switched_model::ComKinoInitializer;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout;

  QuadrupedWheeledInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                            const com_model_t& comModel, const ad_com_model_t& adComModel, const std::string& pathToConfigFolder);

  ~QuadrupedWheeledInterface() override = default;

  const time_triggered_rollout_t& getRollout() const override { return *timeTriggeredRolloutPtr_; };

  const synchronized_module_ptr_array_t& getSynchronizedModules() const override { return solverModules_; };

  const system_dynamics_t& getDynamics() const override { return *dynamicsPtr_; }

  const cost_function_t& getCost() const override { return *costFunctionPtr_; }

  const ocs2::QuadraticCostFunction* getTerminalCostPtr() const override { return terminalCostFunctionPtr_.get(); }

  const constraint_t* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const initializer_t& getInitializer() const override { return *initializerPtr_; }

 private:
  std::unique_ptr<system_dynamics_t> dynamicsPtr_;
  std::unique_ptr<constraint_t> constraintsPtr_;
  std::unique_ptr<cost_function_t> costFunctionPtr_;
  std::unique_ptr<ocs2::QuadraticCostFunction> terminalCostFunctionPtr_;
  std::unique_ptr<initializer_t> initializerPtr_;
  std::unique_ptr<time_triggered_rollout_t> timeTriggeredRolloutPtr_;
  synchronized_module_ptr_array_t solverModules_;
};

}  // namespace switched_model
