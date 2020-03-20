//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedPointfootInterface.h"

namespace switched_model {

QuadrupedPointfootInterface::QuadrupedPointfootInterface(const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, pathToConfigFolder) {
  // Swing planner.
  switched_model::SwingTrajectoryPlannerSettings settings{};
  settings.liftOffVelocity = modelSettings().liftOffVelocity_;
  settings.touchDownVelocity = modelSettings().touchDownVelocity_;
  settings.swingHeight = modelSettings().swingLegLiftOff_;
  settings.touchdownAfterHorizon = 0.2;
  settings.errorGain = 5.0;
  settings.swingTimeScale = 0.15;
  auto swingTrajectoryPlanner =
      std::make_shared<SwingTrajectoryPlanner>(settings, getComModel(), getKinematicModel(), getModeScheduleManagerPtr());
  solverModules_.push_back(swingTrajectoryPlanner);

  state_matrix_t Q;
  input_matrix_t R;
  state_matrix_t Qfinal;
  std::tie(Q, R, Qfinal) = loadCostMatrices(pathToConfigFolder + "/task.info", getKinematicModel(), getInitialState());
  costFunctionPtr_.reset(new cost_function_t(getComModel(), getModeScheduleManagerPtr(), Q, R, Qfinal));

  dynamicsPtr_.reset(new system_dynamics_t(adKinematicModel, adComModel, modelSettings().recompileLibraries_));
  dynamicsDerivativesPtr_.reset(dynamicsPtr_->clone());
  constraintsPtr_.reset(
      new constraint_t(adKinematicModel, adComModel, getModeScheduleManagerPtr(), swingTrajectoryPlanner, modelSettings()));
  operatingPointsPtr_.reset(new operating_point_t(getComModel(), getModeScheduleManagerPtr()));
  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, rolloutSettings()));
}

}  // namespace switched_model
