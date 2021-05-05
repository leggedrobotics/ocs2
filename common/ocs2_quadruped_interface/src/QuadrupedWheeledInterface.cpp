//
// Created by rgrandia on 19.03.20.
//

#include "ocs2_quadruped_interface/QuadrupedWheeledInterface.h"

#include "ocs2_ddp/ContinuousTimeLqr.h"

namespace switched_model {

QuadrupedWheeledInterface::QuadrupedWheeledInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                                     const com_model_t& comModel, const ad_com_model_t& adComModel,
                                                     const std::string& pathToConfigFolder)
    : QuadrupedInterface(kinematicModel, adKinematicModel, comModel, adComModel, pathToConfigFolder) {
  costFunctionPtr_.reset(new SwitchedModelCostBase(costSettings(), *getSwitchedModelModeScheduleManagerPtr(),
                                                   getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), kinematicModel,
                                                   adKinematicModel, comModel, adComModel, modelSettings()));
  dynamicsPtr_.reset(new system_dynamics_t(adKinematicModel, adComModel, modelSettings().recompileLibraries_));
  constraintsPtr_.reset(new constraint_t(adKinematicModel, adComModel, *getSwitchedModelModeScheduleManagerPtr(),
                                         getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), modelSettings()));
  operatingPointsPtr_.reset(new operating_point_t(getComModel(), *getSwitchedModelModeScheduleManagerPtr()));
  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, rolloutSettings()));

  // Initialize cost to be able to query it
  const auto stanceFlags = switched_model::constantFeetArray(true);
  const auto uSystemForWeightCompensation = weightCompensatingInputs(getComModel(), stanceFlags, switched_model::vector3_t::Zero());
  ocs2::CostDesiredTrajectories costDesiredTrajectories({0.0}, {getInitialState()}, {uSystemForWeightCompensation});
  costFunctionPtr_->setCostDesiredTrajectoriesPtr(&costDesiredTrajectories);
  getSwitchedModelModeScheduleManagerPtr()->preSolverRun(0.0, 1.0, getInitialState(), costDesiredTrajectories);

  const auto lqrSolution =
      ocs2::continuous_time_lqr::solve(*dynamicsPtr_, *costFunctionPtr_, 0.0, getInitialState(), uSystemForWeightCompensation);
  terminalCostFunctionPtr_.reset(new ocs2::QuadraticCostFunction(ocs2::matrix_t(), ocs2::matrix_t(), lqrSolution.valueFunction));
}

}  // namespace switched_model
