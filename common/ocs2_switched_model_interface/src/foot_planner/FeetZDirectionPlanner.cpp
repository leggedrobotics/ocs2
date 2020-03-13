/*
 * FeetZDirectionPlanner.h
 *
 *  Created on: Feb 5, 2018
 *      Author: farbod
 */

#include "ocs2_switched_model_interface/foot_planner/FeetZDirectionPlanner.h"

namespace switched_model {

FeetZDirectionPlanner::FeetZDirectionPlanner(scalar_t swingLegLiftOff, scalar_t swingTimeScale, scalar_t liftOffVelocity,
                                             scalar_t touchDownVelocity)
    : swingLegLiftOff_(swingLegLiftOff),
      swingTimeScale_(swingTimeScale),
      liftOffVelocity_(liftOffVelocity),
      touchDownVelocity_(touchDownVelocity),
      phaseIDsStock_(0),
      eventTimes_(0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeetZDirectionPlanner* FeetZDirectionPlanner::clone() const {
  return new FeetZDirectionPlanner(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto FeetZDirectionPlanner::planSingleMode(size_t index, const ocs2::ModeSchedule& modeSchedule) -> feet_cpg_ptr_t {
  const auto& phaseIDsStock = modeSchedule.modeSequence();
  const auto& eventTimes = modeSchedule.eventTimes();

  // update FeetZDirectionPlanner if a new phaseIDsStock is set
  if (phaseIDsStock_ != phaseIDsStock) {
    phaseIDsStock_ = phaseIDsStock;
    eesContactFlagStocks_ = extractContactFlags(phaseIDsStock_);

    for (size_t j = 0; j < NUM_CONTACT_POINTS; j++) {
      std::tie(startTimesIndices_[j], finalTimesIndices_[j]) = updateFootSchedule(j, phaseIDsStock_, eesContactFlagStocks_[j]);
    }
  }

  // update eventTimes
  eventTimes_ = eventTimes;

  feet_cpg_ptr_t feetCpg;
  for (size_t j = 0; j < NUM_CONTACT_POINTS; j++) {
    // create the CPG from the given times
    feetCpg[j].reset(new cpg_t({liftOffVelocity_, touchDownVelocity_}));

    // skip if it is a stance leg
    if (!eesContactFlagStocks_[j][index]) {
      const int swingStartIndex = startTimesIndices_[j][index];
      const int swingFinalIndex = finalTimesIndices_[j][index];
      checkThatIndicesAreValid(j, index, swingStartIndex, swingFinalIndex);

      const scalar_t swingStartTime = eventTimes_[swingStartIndex];
      const scalar_t swingFinalTime = eventTimes_[swingFinalIndex];

      const scalar_t adaptedLiftOff = swingLegLiftOff_ * adaptiveSwingLegLiftOff(swingStartTime, swingFinalTime, swingTimeScale_);

      feetCpg[j]->set({swingStartTime, 0.0}, {swingFinalTime, 0.0}, adaptedLiftOff);
    } else {  // for a stance leg
      feetCpg[j]->set({0.0, 0.0}, {1.0, 0.0}, 0.0);
    }
  }
  return feetCpg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeetZDirectionPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex) const {
  const size_t numSubsystems = phaseIDsStock_.size();
  if (startIndex < 0) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock_[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
  if (finalIndex >= numSubsystems - 1) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock_[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
}

}  // namespace switched_model
