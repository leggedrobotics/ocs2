//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const com_model_t& comModel,
                                               const kinematic_model_t& kinematicsModel,
                                               std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr)
    : settings_(std::move(settings)),
      comModelPtr_(comModel.clone()),
      kinematicModelPtr_(kinematicsModel.clone()),
      modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const -> scalar_t {
  const auto modeIndex = ocs2::lookup::findIndexInTimeArray(eventTimes_, time);
  return feetTrajectoriesPerModePerLeg_[modeIndex][leg]->velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::preSolverRun(scalar_t initTime, scalar_t finalTime, const state_vector_t& currentState,
                                          const ocs2::CostDesiredTrajectories& costDesiredTrajectory) {
  const auto& modeSchedule = modeScheduleManagerPtr_->getModeSchedule();
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);

  std::array<std::vector<int>, 4> startTimesIndices;
  std::array<std::vector<int>, 4> finalTimesIndices;
  for (size_t leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(leg, modeSequence, eesContactFlagStocks[leg]);
  }

  feetTrajectoriesPerModePerLeg_.clear();
  feetTrajectoriesPerModePerLeg_.reserve(modeSequence.size());
  for (int p = 0; p < modeSequence.size(); ++p) {
    std::array<std::unique_ptr<SplineCpg>, 4> feetCpg;

    for (size_t j = 0; j < NUM_CONTACT_POINTS; j++) {
      if (!eesContactFlagStocks[j][p]) {  // for a swing leg
        feetCpg[j].reset(new SplineCpg({settings_.liftOffVelocity, settings_.touchDownVelocity}));
        const int swingStartIndex = startTimesIndices[j][p];
        const int swingFinalIndex = finalTimesIndices[j][p];
        checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex);

        const scalar_t swingStartTime = eventTimes[swingStartIndex];
        const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

        const scalar_t adaptedLiftOff =
            settings_.swingHeight * swingTrajectoryScaling(swingStartTime, swingFinalTime, settings_.swingTimeScale);

        feetCpg[j]->set({swingStartTime, 0.0}, {swingFinalTime, 0.0}, adaptedLiftOff);
      } else {  // for a stance leg
        feetCpg[j].reset(new SplineCpg({0.0, 0.0}));
        feetCpg[j]->set({0.0, 0.0}, {1.0, 0.0}, 0.0);
      }
    }
    feetTrajectoriesPerModePerLeg_.push_back(std::move(feetCpg));
  }

  // Store eventTimes
  eventTimes_ = eventTimes;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<std::vector<int>, std::vector<int>> SwingTrajectoryPlanner::updateFootSchedule(size_t footIndex,
                                                                                         const std::vector<size_t>& phaseIDsStock,
                                                                                         const std::vector<bool>& contactFlagStock) const {
  const size_t numPhases = phaseIDsStock.size();

  std::vector<int> startTimeIndexStock(numPhases, 0);
  std::vector<int> finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing feet
  for (size_t i = 0; i < numPhases; i++) {
    if (!contactFlagStock[i]) {
      std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
    }
  }
  return {startTimeIndexStock, finalTimeIndexStock};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::array<std::vector<bool>, NUM_CONTACT_POINTS> SwingTrajectoryPlanner::extractContactFlags(
    const std::vector<size_t>& phaseIDsStock) const {
  const size_t numPhases = phaseIDsStock.size();

  std::array<std::vector<bool>, NUM_CONTACT_POINTS> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    std::array<bool, NUM_CONTACT_POINTS> contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < NUM_CONTACT_POINTS; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock) const {
  const size_t numPhases = contactFlagStock.size();

  // skip if it is a stance leg
  if (contactFlagStock[index]) {
    return {0, 0};
  }

  // find the starting time
  int startTimesIndex = -1;
  for (int ip = index - 1; ip >= 0; ip--) {
    if (contactFlagStock[ip]) {
      startTimesIndex = ip;
      break;
    }
  }

  // find the final time
  int finalTimesIndex = numPhases - 1;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (contactFlagStock[ip]) {
      finalTimesIndex = ip - 1;
      break;
    }
  }

  return {startTimesIndex, finalTimesIndex};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale) const -> scalar_t {
  return std::min(1.0, (finalTime - startTime) / swingTimeScale);
}

}  // namespace switched_model
