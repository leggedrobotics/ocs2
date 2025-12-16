/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_switched_model_interface/logic/ModeSequenceTemplate.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>

namespace switched_model {

std::ostream& operator<<(std::ostream& stream,
                         const ModeSequenceTemplate& ModeSequenceTemplate) {
  stream << "Template switching times: {"
         << ocs2::toDelimitedString(ModeSequenceTemplate.switchingTimes)
         << "}\n";
  stream << "Template mode sequence:   {"
         << ocs2::toDelimitedString(ModeSequenceTemplate.modeSequence) << "}\n";
  return stream;
}

ModeSequenceTemplate loadModeSequenceTemplate(const std::string& filename,
                                              const std::string& topicName,
                                              bool verbose) {
  std::vector<scalar_t> switchingTimes;
  ocs2::loadData::loadStdVector(filename, topicName + ".switchingTimes",
                                switchingTimes, verbose);

  std::vector<std::string> modeSequenceString;
  ocs2::loadData::loadStdVector(filename, topicName + ".modeSequence",
                                modeSequenceString, verbose);

  if (switchingTimes.empty() || modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " +
                             topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(string2ModeNumber(modeName));
  }

  return {switchingTimes, modeSequence};
}

ocs2_msgs::msg::ModeSchedule createModeSequenceTemplateMsg(
    const ModeSequenceTemplate& ModeSequenceTemplate) {
  ocs2_msgs::msg::ModeSchedule modeScheduleMsg;
  modeScheduleMsg.event_times.assign(
      ModeSequenceTemplate.switchingTimes.begin(),
      ModeSequenceTemplate.switchingTimes.end());
  modeScheduleMsg.mode_sequence.assign(
      ModeSequenceTemplate.modeSequence.begin(),
      ModeSequenceTemplate.modeSequence.end());
  return modeScheduleMsg;
}

ModeSequenceTemplate readModeSequenceTemplateMsg(
    const ocs2_msgs::msg::ModeSchedule& modeScheduleMsg) {
  std::vector<scalar_t> switchingTimes(modeScheduleMsg.event_times.begin(),
                                       modeScheduleMsg.event_times.end());
  std::vector<size_t> modeSequence(modeScheduleMsg.mode_sequence.begin(),
                                   modeScheduleMsg.mode_sequence.end());
  return {switchingTimes, modeSequence};
}

Gait toGait(const ModeSequenceTemplate& ModeSequenceTemplate) {
  const auto startTime = ModeSequenceTemplate.switchingTimes.front();
  const auto endTime = ModeSequenceTemplate.switchingTimes.back();
  Gait gait;
  gait.duration = endTime - startTime;
  // Events: from time -> phase
  gait.eventPhases.reserve(ModeSequenceTemplate.switchingTimes.size());
  std::for_each(
      ModeSequenceTemplate.switchingTimes.begin() + 1,
      ModeSequenceTemplate.switchingTimes.end() - 1, [&](scalar_t eventTime) {
        gait.eventPhases.push_back((eventTime - startTime) / gait.duration);
      });
  // Modes:
  gait.modeSequence = ModeSequenceTemplate.modeSequence;
  assert(isValidGait(gait));
  return gait;
}

ocs2::ModeSchedule loadModeSchedule(const std::string& filename,
                                    const std::string& topicName,
                                    bool verbose) {
  std::vector<scalar_t> eventTimes;
  ocs2::loadData::loadStdVector(filename, topicName + ".eventTimes", eventTimes,
                                verbose);

  std::vector<std::string> modeSequenceString;
  ocs2::loadData::loadStdVector(filename, topicName + ".modeSequence",
                                modeSequenceString, verbose);

  if (modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSchedule] failed to load : " +
                             topicName + " from " + filename);
  }

  // convert the mode name to mode enum
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeSequenceString.size());
  for (const auto& modeName : modeSequenceString) {
    modeSequence.push_back(string2ModeNumber(modeName));
  }

  return {eventTimes, modeSequence};
}

}  // namespace switched_model
