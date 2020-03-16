//
// Created by rgrandia on 15.03.20.
//

#include "ocs2_switched_model_interface/logic/GaitSchedule.h"

#include "ocs2_switched_model_interface/logic/GaitSwitching.h"

namespace switched_model {

GaitSchedule::GaitSchedule(double time, Gait gait) : time_(time), phase_(0.0), gaitSchedule_{std::move(gait)} {}

void GaitSchedule::advanceToTime(double time) {
  assert(time >= time_);

  const double dt = time - time_;
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

void GaitSchedule::setGaitAtTime(const Gait& gait, double time) {
  setGaitSequenceAtTime({gait}, time);
}

void GaitSchedule::setGaitSequenceAtTime(const std::vector<Gait>& gaitSequence, double time) {
  assert(time >= time_);

  rolloutGaitScheduleTillTime(time);

  double newPhase;
  std::deque<Gait>::iterator newActiveGait;
  std::tie(newPhase, newActiveGait) = advancePhase(phase_, time - time_, gaitSchedule_.begin(), gaitSchedule_.end());

  // Shrink the gait that is active at "time", s.t. it ends at "time"
  newActiveGait->duration *= newPhase;

  gaitSchedule_.erase(newActiveGait + 1, gaitSchedule_.end());
  gaitSchedule_.insert(gaitSchedule_.end(), gaitSequence.begin(), gaitSequence.end());
}

ocs2::ModeSchedule GaitSchedule::getModeSchedule(double timeHorizon) const {
  return ::switched_model::getModeSchedule(phase_, time_, timeHorizon, gaitSchedule_.begin(), gaitSchedule_.end());
}

void GaitSchedule::rolloutGaitScheduleTillTime(double time) {
  double t = time_;
  auto gaitIt = gaitSchedule_.begin();
  while (t < time) {
    if (gaitIt == gaitSchedule_.end()) {
      // End of the schedule reached: make the repetition of the last gait explicit
      gaitSchedule_.push_back(gaitSchedule_.back());
      gaitIt = std::prev(gaitSchedule_.end());
    }
    t += gaitIt->duration;
    ++gaitIt;
  }
}

}