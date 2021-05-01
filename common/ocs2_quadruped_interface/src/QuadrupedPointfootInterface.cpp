//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedPointfootInterface.h"

namespace switched_model {

QuadrupedPointfootInterface::QuadrupedPointfootInterface(const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, pathToConfigFolder) {
  costFunctionPtr_.reset(new SwitchedModelCostBase(costSettings(), *getSwitchedModelModeScheduleManagerPtr(), kinematicModel,
                                                   adKinematicModel, comModel, modelSettings().recompileLibraries_));

  state_matrix_t Qfinal = state_matrix_t::Zero();
  terminalCostFunctionPtr_.reset(new ocs2::QuadraticCostFunction(ocs2::matrix_t(), ocs2::matrix_t(), Qfinal));

  dynamicsPtr_.reset(new ComKinoSystemDynamicsAd(adKinematicModel, adComModel, modelSettings().recompileLibraries_));
  constraintsPtr_.reset(new ComKinoConstraintBaseAd(adKinematicModel, adComModel, *getSwitchedModelModeScheduleManagerPtr(),
                                                    getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(),
                                                    modelSettings()));
  operatingPointsPtr_.reset(new ComKinoOperatingPointsBase(getComModel(), *getSwitchedModelModeScheduleManagerPtr()));
  timeTriggeredRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings()));
}

}  // namespace switched_model
