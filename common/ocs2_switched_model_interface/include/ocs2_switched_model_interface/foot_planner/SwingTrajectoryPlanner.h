//
// Created by rgrandia on 18.03.20.
//

#pragma once

#include <ocs2_core/Dimensions.h>

#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SplineCpg.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

struct SwingTrajectoryPlannerSettings {
  using scalar_t = ocs2::Dimensions<0, 0>::scalar_t;
  scalar_t liftOffVelocity;
  scalar_t touchDownVelocity;
  scalar_t swingHeight;
  scalar_t swingTimeScale;  // swing phases shorter than this time will be scaled down in height and velocity
};

class SwingTrajectoryPlanner : public ocs2::SolverSynchronizedModule<STATE_DIM, INPUT_DIM> {
  using scalar_t = ocs2::Dimensions<0, 0>::scalar_t;
  using com_model_t = ComModelBase<scalar_t>;
  using kinematic_model_t = KinematicsModelBase<scalar_t>;

 public:
  SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const com_model_t& comModel, const kinematic_model_t& kinematicsModel,
                         std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr);

  scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                    const ocs2::CostDesiredTrajectories& costDesiredTrajectory) override;

  void postSolverRun(const primal_solution_t& primalSolution) override {}

 private:
  /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
  std::array<std::vector<bool>, NUM_CONTACT_POINTS> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

  /**
   * Finds the take-off and touch-down times indices for a specific leg.
   *
   * @param index
   * @param contactFlagStock
   * @return {The take-off time index for swing legs, touch-down time index for swing legs}
   */
  std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock) const;

  /**
   * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
   * phases of the a foot in each subsystem.
   *
   * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
   * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
   *
   * @param [in] footIndex: Foot index
   * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
   * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
   * @return { startTimeIndexStock, finalTimeIndexStock}
   */
  std::pair<std::vector<int>, std::vector<int>> updateFootSchedule(size_t footIndex, const std::vector<size_t>& phaseIDsStock,
                                                                   const std::vector<bool>& contactFlagStock) const;

  /**
   * Check if event time indices are valid
   * @param leg
   * @param index : phase index
   * @param startIndex : liftoff event time index
   * @param finalIndex : touchdown event time index
   * @param phaseIDsStock : mode sequence
   */
  void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex, const std::vector<size_t>& phaseIDsStock) const;

  scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale) const;

  SwingTrajectoryPlannerSettings settings_;

  std::vector<std::array<std::unique_ptr<SplineCpg>, 4>> feetTrajectoriesPerModePerLeg_;
  std::vector<scalar_t> eventTimes_;

  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;
  std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
};

}  // namespace switched_model
