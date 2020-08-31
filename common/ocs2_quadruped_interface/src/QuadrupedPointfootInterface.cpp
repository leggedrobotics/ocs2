//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedPointfootInterface.h"

namespace switched_model {

QuadrupedPointfootInterface::QuadrupedPointfootInterface(const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, pathToConfigFolder) {
  state_matrix_t Q;
  input_matrix_t R;
  state_matrix_t Qfinal;
  std::tie(Q, R, Qfinal) = loadCostMatrices(pathToConfigFolder + "/task.info", getKinematicModel(), getInitialState());
  costFunctionPtr_.reset(new SwitchedModelCostBase(getComModel(), *getModeScheduleManagerPtr(), Q, R, Qfinal));

  dynamicsPtr_.reset(new ComKinoSystemDynamicsAd(adKinematicModel, adComModel, modelSettings().recompileLibraries_));
  constraintsPtr_.reset(new ComKinoConstraintBaseAd(adKinematicModel, adComModel, *getModeScheduleManagerPtr(),
                                                    getModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), modelSettings()));
  operatingPointsPtr_.reset(new ComKinoOperatingPointsBase(getComModel(), *getModeScheduleManagerPtr()));
  timeTriggeredRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings()));
}

}  // namespace switched_model
