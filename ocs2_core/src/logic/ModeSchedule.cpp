//
// Created by rgrandia on 09.03.20.
//

#include "ocs2_core/logic/ModeSchedule.h"

#include <ocs2_core/misc/Display.h>

namespace ocs2 {

ModeSchedule::ModeSchedule(std::vector<scalar_t> eventTimes, std::vector<size_t> modeSequence)
    : eventTimes_(std::move(eventTimes)), modeSequence_(std::move(modeSequence)) {
  assert(eventTimes_.size() == modeSequence.size() - 1);
}

std::string display(const ModeSchedule& modeSchedule) {
  std::stringstream ss;
  ss << "switching times: \t {" << toDelimitedString(modeSchedule.eventTimes()) << "}\n";
  ss << "mode sequence: \t {" << toDelimitedString(modeSchedule.modeSequence()) << "}\n";
  return ss.str();
}

}