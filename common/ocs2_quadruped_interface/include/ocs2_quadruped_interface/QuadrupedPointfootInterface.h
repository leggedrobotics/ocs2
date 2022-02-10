//
// Created by rgrandia on 19.03.20.
//

#pragma once

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_switched_model_interface/initialization/ComKinoInitializer.h>

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

namespace switched_model {

class QuadrupedPointfootInterface : public QuadrupedInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using QuadrupedInterface::synchronized_module_ptr_array_t;

  QuadrupedPointfootInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                              const com_model_t& comModel, const ad_com_model_t& adComModel, Settings settings);

  ~QuadrupedPointfootInterface() override = default;

  const ocs2::TimeTriggeredRollout& getRollout() const override { return *timeTriggeredRolloutPtr_; };

  const synchronized_module_ptr_array_t& getSynchronizedModules() const override { return solverModules_; };

  const ComKinoInitializer& getInitializer() const override { return *initializerPtr_; }

  const ScalarFunctionQuadraticApproximation& nominalCostApproximation() const override { return nominalCostApproximation_; }

 private:
  std::unique_ptr<ComKinoInitializer> initializerPtr_;
  std::unique_ptr<ocs2::TimeTriggeredRollout> timeTriggeredRolloutPtr_;
  synchronized_module_ptr_array_t solverModules_;
  ScalarFunctionQuadraticApproximation nominalCostApproximation_;
};

}  // namespace switched_model
