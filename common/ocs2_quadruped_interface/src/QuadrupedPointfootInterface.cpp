//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedPointfootInterface.h"

#include "ocs2_ddp/ContinuousTimeLqr.h"

namespace switched_model {

QuadrupedPointfootInterface::QuadrupedPointfootInterface(const kinematic_model_t& kinematicModel,
                                                         const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                                                         const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, pathToConfigFolder) {
  costFunctionPtr_.reset(new SwitchedModelCostBase(costSettings(), *getSwitchedModelModeScheduleManagerPtr(), kinematicModel,
                                                   adKinematicModel, comModel, modelSettings().recompileLibraries_));
  dynamicsPtr_.reset(new ComKinoSystemDynamicsAd(adKinematicModel, adComModel, *getSwitchedModelModeScheduleManagerPtr(),
                                                 modelSettings().recompileLibraries_));
  constraintsPtr_.reset(new ComKinoConstraintBaseAd(adKinematicModel, adComModel, *getSwitchedModelModeScheduleManagerPtr(),
                                                    getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(),
                                                    modelSettings()));
  initializerPtr_.reset(new ComKinoInitializer(getComModel(), *getSwitchedModelModeScheduleManagerPtr()));
  timeTriggeredRolloutPtr_.reset(new ocs2::TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings()));

  // Initialize cost to be able to query it
  const auto stanceFlags = switched_model::constantFeetArray(true);
  const auto uSystemForWeightCompensation = weightCompensatingInputs(getComModel(), stanceFlags, switched_model::vector3_t::Zero());
  ocs2::TargetTrajectories targetTrajectories({0.0}, {getInitialState()}, {uSystemForWeightCompensation});
  costFunctionPtr_->setTargetTrajectoriesPtr(&targetTrajectories);

  const auto lqrSolution =
      ocs2::continuous_time_lqr::solve(*dynamicsPtr_, *costFunctionPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);
  terminalCostFunctionPtr_.reset(new ocs2::QuadraticCostFunction(ocs2::matrix_t(), ocs2::matrix_t(), lqrSolution.valueFunction));
}

}  // namespace switched_model
