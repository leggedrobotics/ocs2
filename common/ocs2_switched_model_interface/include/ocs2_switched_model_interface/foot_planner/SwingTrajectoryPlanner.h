//
// Created by rgrandia on 13.03.20.
//

#pragma once

#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SplineCpg.h"

namespace switched_model {

struct SwingTrajectoryPlannerSettings {
  double liftOffVelocity;
  double touchDownVelocity;
  double swingHeight;
  double touchdownAfterHorizon;  // swing time added beyond the horizon if there is no touchdown in the current mode schedule
  double errorGain;              // proportional gain for returning to the planned swing trajectory
};

class SwingTrajectoryPlanner : public ocs2::SolverSynchronizedModule<STATE_DIM, INPUT_DIM> {
 public:
  SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const ComModelBase<double>& comModel,
                         const KinematicsModelBase<double>& kinematicsModel);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory, const ocs2::ModeSchedule& modeSchedule) override;

  void postSolverRun(const primal_solution_t& primalSolution) override {}

  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

 private:
  void updateLastContacts(scalar_t time, const std::array<vector3_t, NUM_CONTACT_POINTS>& currentFeetPositions,
                          const ocs2::ModeSchedule& modeSchedule);
  void updateFeetTrajectories(scalar_t finalTime, const ocs2::ModeSchedule& modeSchedule);
  void updateErrorTrajectories(scalar_t initTime, const std::array<vector3_t, NUM_CONTACT_POINTS>& currentFeetPositions,
                               const ocs2::ModeSchedule& modeSchedule);

  SwingTrajectoryPlannerSettings settings_;
  std::unique_ptr<ComModelBase<double>> comModel_;
  std::unique_ptr<KinematicsModelBase<double>> kinematicsModel_;

  std::array<SplineCpg::Point, NUM_CONTACT_POINTS> lastContacts_;
  std::array<std::vector<SplineCpg>, NUM_CONTACT_POINTS> feetHeightTrajectories_;
  std::vector<scalar_t> eventTimes_;

  // Error correction
  scalar_t initTime_;
  std::array<scalar_t, NUM_CONTACT_POINTS> initialErrors_;
};

}  // namespace switched_model
