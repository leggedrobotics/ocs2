//
// Created by rgrandia on 15.03.20.
//

#include "ocs2_switched_model_interface/logic/GaitSchedule.h"

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/logic/GaitSwitching.h"

namespace switched_model {

GaitSchedule::GaitSchedule(scalar_t time, Gait gait) : time_(time), phase_(0.0), gaitSchedule_{std::move(gait)} {}

void GaitSchedule::advanceToTime(scalar_t time) {
  assert(time >= time_);

  const scalar_t dt = time - time_;
  std::deque<Gait>::iterator newActiveGait;
  std::tie(phase_, newActiveGait) = advancePhase(phase_, dt, gaitSchedule_.begin(), gaitSchedule_.end());
  time_ = time;

  // Remove gaits that have been completed.
  while (newActiveGait != gaitSchedule_.begin()) {
    gaitSchedule_.pop_front();
  }
}

void GaitSchedule::setNextGait(const Gait& gait) {
  setGaitSequenceAfterCurrentGait({gait});
}

void GaitSchedule::setGaitSequenceAfterCurrentGait(const std::vector<Gait>& gaitSequence) {
  gaitSchedule_.erase(gaitSchedule_.begin() + 1, gaitSchedule_.end());
  gaitSchedule_.insert(gaitSchedule_.end(), gaitSequence.begin(), gaitSequence.end());
}

void GaitSchedule::setGaitAtTime(const Gait& gait, scalar_t time) {
  setGaitSequenceAtTime({gait}, time);
}

void GaitSchedule::setGaitSequenceAtTime(const std::vector<Gait>& gaitSequence, scalar_t time) {
  assert(time >= time_);

  rolloutGaitScheduleTillTime(time);

  scalar_t newPhase;
  std::deque<Gait>::iterator newActiveGait;
  std::tie(newPhase, newActiveGait) = advancePhase(phase_, time - time_, gaitSchedule_.begin(), gaitSchedule_.end());

  // Shrink the gait that is active at "time", s.t. it ends at "time"
  if (newActiveGait == gaitSchedule_.begin()) {
    // Gait to shrink is already active. Determine new duration from the amount of time and phase left.
    newActiveGait->duration = (time - time_) / (1.0 - phase_);
  } else {
    newActiveGait->duration *= newPhase;
  }

  gaitSchedule_.erase(newActiveGait + 1, gaitSchedule_.end());
  gaitSchedule_.insert(gaitSchedule_.end(), gaitSequence.begin(), gaitSequence.end());
}

void GaitSchedule::setGaitAfterTime(const Gait& gait, scalar_t time) {
  setGaitSequenceAfterTime({gait}, time);
}

void GaitSchedule::setGaitSequenceAfterTime(const std::vector<Gait>& gaitSequence, scalar_t time) {
  assert(time >= time_);

  rolloutGaitScheduleTillTime(time);

  scalar_t newPhase;
  std::deque<Gait>::iterator newActiveGait;
  std::tie(newPhase, newActiveGait) = advancePhase(phase_, time - time_, gaitSchedule_.begin(), gaitSchedule_.end());

  gaitSchedule_.erase(newActiveGait + 1, gaitSchedule_.end());
  gaitSchedule_.insert(gaitSchedule_.end(), gaitSequence.begin(), gaitSequence.end());
}

ocs2::ModeSchedule GaitSchedule::getModeSchedule(scalar_t timeHorizon) const {
  return ::switched_model::getModeSchedule(phase_, time_, timeHorizon, gaitSchedule_.begin(), gaitSchedule_.end());
}

void GaitSchedule::rolloutGaitScheduleTillTime(scalar_t time) {
  scalar_t tGaitEnd = time_ + timeLeftInGait(getCurrentPhase(), getCurrentGait());
  auto gaitIt = gaitSchedule_.begin();
  while (tGaitEnd < time) {
    if (std::next(gaitIt) == gaitSchedule_.end()) {
      // End of the schedule reached: make the repetition of the last gait explicit
      gaitSchedule_.push_back(gaitSchedule_.back());
      gaitIt = std::prev(gaitSchedule_.end(), 2);  // Iterator can be invalidated after push_back, so reset set explicitly.
    }
    tGaitEnd += gaitIt->duration;
    ++gaitIt;
  }
}

bool isStandingDuringTimeHorizon(scalar_t timeHorizon, const GaitSchedule& gaitSchedule) {
  const auto modeSchedule = gaitSchedule.getModeSchedule(timeHorizon);
  return std::all_of(modeSchedule.modeSequence.begin(), modeSchedule.modeSequence.end(),
                     [](size_t mode) { return mode == ModeNumber::STANCE; });
}

scalar_t getTimeUntilNextLiftOff(scalar_t timeHorizon, size_t legId, const GaitSchedule& gaitSchedule) {
  const auto modeSchedule = gaitSchedule.getModeSchedule(timeHorizon);
  scalar_t timeUntilNextLiftOff = 0.0;
  unsigned int modeId = 0;
  for (const auto mode : modeSchedule.modeSequence) {
    std::array<bool, 4> stanceLegs = modeNumber2StanceLeg(mode);
    if (!stanceLegs.at(legId)) {
      return timeUntilNextLiftOff - gaitSchedule.getCurrentTime();
    } else if (modeId < modeSchedule.eventTimes.size()) {
      timeUntilNextLiftOff = modeSchedule.eventTimes.at(modeId);
      ++modeId;
    } else {
      return -1.0;
    }
  }
}

scalar_t getTimeUntilNextTouchDown(scalar_t timeHorizon, size_t legId, const GaitSchedule& gaitSchedule) {
  const auto modeSchedule = gaitSchedule.getModeSchedule(timeHorizon);
  unsigned int modeId = 0;
  bool isSwinging = false;
  for (const auto mode : modeSchedule.modeSequence) {
    std::array<bool, 4> stanceLegs = switched_model::modeNumber2StanceLeg(mode);
    if (!stanceLegs.at(legId)) {
      isSwinging = true;
    } else if (isSwinging) {
      if (modeId < modeSchedule.eventTimes.size()) {
        return modeSchedule.eventTimes.at(modeId - 1) - gaitSchedule.getCurrentTime();
      } else {
        return -1.0;
      }
    }
    ++modeId;
  }
  return -1.0;
}

}  // namespace switched_model