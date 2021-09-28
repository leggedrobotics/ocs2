//
// Created by rgrandia on 19.03.20.
//

#pragma once

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_switched_model_interface/initialization/ComKinoInitializer.h>

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

namespace switched_model {

class QuadrupedWheeledInterface : public QuadrupedInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using initializer_t = switched_model::ComKinoInitializer;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout;

  QuadrupedWheeledInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                            const com_model_t& comModel, const ad_com_model_t& adComModel, const std::string& pathToConfigFolder);

  ~QuadrupedWheeledInterface() override = default;

  const time_triggered_rollout_t& getRollout() const override { return *timeTriggeredRolloutPtr_; };

  const synchronized_module_ptr_array_t& getSynchronizedModules() const override { return solverModules_; };

  const initializer_t& getInitializer() const override { return *initializerPtr_; }

 private:
  std::unique_ptr<initializer_t> initializerPtr_;
  std::unique_ptr<time_triggered_rollout_t> timeTriggeredRolloutPtr_;
  synchronized_module_ptr_array_t solverModules_;
};

}  // namespace switched_model
