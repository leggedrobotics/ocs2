/*
 * GaitAdaptation.h
 *
 *  Created on: Jun 4, 2020
 *      Author: Marko Bjelonic
 */

#pragma once

#include <ocs2_switched_model_interface/logic/Gait.h>
#include <ocs2_switched_model_interface/logic/GaitSchedule.h>

namespace switched_model {

struct GaitAdaptationSettings {
  scalar_t earlyTouchDownTimeWindow = 0.1;
};

class GaitAdaptation {
 public:
  GaitAdaptation(const GaitAdaptationSettings& settings, const contact_flag_t& measuredContactFlags);

  void advance(GaitSchedule& gaitSchedule, const contact_flag_t& measuredContactFlags, scalar_t dt);

 private:
  /// Update measured liftoff information
  void advanceLiftoffTracking(const contact_flag_t& desiredContactFlags, const contact_flag_t& measuredContactFlags);

  /// Update next touchdown and liftoff information
  void advanceSwingEvents(const GaitSchedule& gaitSchedule);

  enum class ScheduledAdaptation { None, EarlyContact };
  static bool isEarlyContact(ScheduledAdaptation adaptation) { return adaptation == ScheduledAdaptation::EarlyContact; }

  /**
   * Determines the gait adaptation strategy per leg.
   * @param desiredContactFlags : true = this leg was planned to be in contact, false = this was planned to be in swing.
   * @param measuredContactFlags : true = this leg is measured to be in contact, false = this leg is measured to be in swing.
   * @return Scheduled adaptation enum per leg.
   */
  feet_array_t<ScheduledAdaptation> advanceLegStrategies(const contact_flag_t& desiredContactFlags,
                                                         const contact_flag_t& measuredContactFlags);
  ScheduledAdaptation desiredContactMeasuredContact(size_t leg);
  ScheduledAdaptation desiredContactMeasuredMotion(size_t leg);
  ScheduledAdaptation desiredMotionMeasuredContact(size_t leg);
  ScheduledAdaptation desiredMotionMeasuredMotion(size_t leg);

  void applyAdaptation(GaitSchedule& gaitSchedule, const feet_array_t<ScheduledAdaptation>& scheduledAdaptations);

  /// Planned event timing relative to the current time (before applying the gait adaptation).
  feet_array_t<scalar_t> timeUntilNextTouchDownPerLeg_;
  feet_array_t<scalar_t> timeUntilNextLiftOffPerLeg_;

  feet_array_t<bool> hasLiftedSinceLastContact_;

  GaitAdaptationSettings settings_;
};

/**
 * Removes the first swing phase for all legs flagged with earlyTouchdown
 */
std::function<void(scalar_t& currentPhase, switched_model::Gait& currentGait, scalar_t currTime, switched_model::Gait& nextGait)>
earlyTouchDownAdaptation(const switched_model::feet_array_t<bool>& earlyTouchDownPerLeg);

/** Gets the mode index of the next phase with the specified contact state, returns the size of the modesequence of no such contact state
 * was found */
int getModeIndexOfNextContactStateOfLeg(bool contact, int startModeIdx, size_t leg, const Gait& gait);

/** Adapts the mode sequence between the two mode ids of a specific leg to be with the specified flag. */
void setContactStateOfLegBetweenModes(bool contact, int startModeIdx, int lastModeIdx, size_t leg, Gait& gait);

}  // namespace switched_model
