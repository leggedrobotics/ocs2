//
// Created by rgrandia on 15.03.20.
//

#pragma once

#include <deque>
#include <vector>

#include <ocs2_core/logic/ModeSchedule.h>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/Gait.h"

namespace switched_model {

class GaitSchedule {
 public:
  GaitSchedule(scalar_t time, Gait gait);

  /** Advances the gait schedule to a specified time, the specified time must be increasing w.r.t. previous calls. */
  void advanceToTime(scalar_t time);

  /** Adds gaits after completing the current gait */
  void setNextGait(const Gait& gait);

  /** Adds sequence of gaits after completing the current gait */
  void setGaitSequenceAfterCurrentGait(const std::vector<Gait>& gaitSequence);

  /** Adds a gait at a specified time. The gait originally active at that time is shrunk in duration to make the new gait fit. Gaits after
   * that time are removed */
  void setGaitAtTime(const Gait& gait, scalar_t time);

  /** Adds schedule of gaits at a specified time. The gait originally active at that time is shrunk in duration to make the new gait fit.
   * Gaits after that time are removed */
  void setGaitSequenceAtTime(const std::vector<Gait>& gaitSequence, scalar_t time);

  /** Adds a gait at the first opportunity after the specified time. Does not adapt gait cycle durations */
  void setGaitAfterTime(const Gait& gait, scalar_t time);

  /** Adds a gait sequence  at the first opportunity after the specified time. Does not adapt gait cycle durations */
  void setGaitSequenceAfterTime(const std::vector<Gait>& gaitSequence, scalar_t time);

  /** Gets the gaitSchedule as a mode schedule from the current time, and for the specified horizon */
  ocs2::ModeSchedule getModeSchedule(scalar_t timeHorizon) const;

  /** Gets the currently active gait */
  const Gait& getCurrentGait() const { return gaitSchedule_.front(); }

  /** Gets phase variable for the current gait */
  scalar_t getCurrentPhase() const { return phase_; }

  /** Gets time variable for the current gait */
  scalar_t getCurrentTime() const { return time_; }

  /** Gets the currently active mode */
  size_t getCurrentMode() const { return getModeFromPhase(getCurrentPhase(), getCurrentGait()); }

  /** Returns time until current gait ends a cycle */
  scalar_t timeLeftInCurrentGait() const { return timeLeftInGait(getCurrentPhase(), getCurrentGait()); }

 private:
  /** Makes the implicit looping of the last gait explicit until the specified time */
  void rolloutGaitScheduleTillTime(scalar_t time);

  scalar_t time_;
  scalar_t phase_;
  std::deque<Gait> gaitSchedule_;
};

bool isStandingDuringTimeHorizon(scalar_t timeHorizon, const GaitSchedule& gaitSchedule);

}  // namespace switched_model
