//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedWheeledInterface.h"

namespace switched_model {

QuadrupedWheeledInterface::QuadrupedWheeledInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                                     const com_model_t& comModel, const ad_com_model_t& adComModel,
                                                     const std::string& pathToConfigFolder)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, pathToConfigFolder) {
  state_matrix_t Q;
  input_matrix_t R;
  state_matrix_t Qfinal;
  std::tie(Q, R, Qfinal) = loadCostMatrices(pathToConfigFolder + "/task.info", getKinematicModel(), getInitialState());
  costFunctionPtr_.reset(new SwitchedModelCostBase(getComModel(), *getSwitchedModelModeScheduleManagerPtr(), Q, R));
  terminalCostFunctionPtr_.reset(new ocs2::QuadraticCostFunction(ocs2::matrix_t(), ocs2::matrix_t(), Qfinal));

  dynamicsPtr_.reset(new system_dynamics_t(adKinematicModel, adComModel, modelSettings().recompileLibraries_));
  constraintsPtr_.reset(new constraint_t(adKinematicModel, adComModel, *getSwitchedModelModeScheduleManagerPtr(),
                                         getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), modelSettings()));
  initializerPtr_.reset(new initializer_t(getComModel(), *getSwitchedModelModeScheduleManagerPtr()));
  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, rolloutSettings()));
}

}  // namespace switched_model
