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

#include "ocs2_ddp/trajectory_adjustment/TrajectorySpreading.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>

#include <ocs2_core/NumericTraits.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

std::ostream& operator<<(std::ostream& os, const std::pair<int, int>& ind) {
  os << "(" << ind.first << ", " << ind.second << ")";
  return os;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TrajectorySpreading::set(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule,
                              const scalar_array_t& oldTimeTrajectory, const size_array_t& oldPostEventIndices) {
  const scalar_t oldInitTime = oldTimeTrajectory.front();
  const scalar_t oldFinalTime = oldTimeTrajectory.back();

  const int oldFirstActiveModeIndex = upperBoundIndex(oldModeSchedule.eventTimes, oldInitTime);
  // sizeof(eventTimes) + 1 == sizeof(modeSequence)
  const int oldLastActiveModeIndex = upperBoundIndex(oldModeSchedule.eventTimes, oldFinalTime);

  const int newFirstActiveModeIndex = upperBoundIndex(newModeSchedule.eventTimes, oldInitTime);
  const int newLastActiveModeIndex = upperBoundIndex(newModeSchedule.eventTimes, oldFinalTime);

  auto oldStartIndexOfMatchedSequence = oldFirstActiveModeIndex;
  // initial active modes. The matched mode sequence for the new ModeSchedule always started from the mode that contains the newInitTime
  const auto newStartIndexOfMatchedSequence = newFirstActiveModeIndex;
  // the size of the window where the active modes are identical.
  // this means: mode[firstMatchingModeIndex + w] != updatedMode[updatedFirstMatchingModeIndex + w]
  size_t w = 0;
  while (oldStartIndexOfMatchedSequence <= oldModeSchedule.modeSequence.size() - 1) {
    // +1 to include the last mode

    assert(oldLastActiveModeIndex + 1 <= oldModeSchedule.modeSequence.size());
    // TODO: change to mismatch(first1, last1, first2, last2). It is supported since c++ 201103
    auto mismatchedIndex = std::mismatch(oldModeSchedule.modeSequence.begin() + oldStartIndexOfMatchedSequence,
                                         oldModeSchedule.modeSequence.begin() + oldLastActiveModeIndex + 1,
                                         newModeSchedule.modeSequence.begin() + newStartIndexOfMatchedSequence);

    while (mismatchedIndex.second - newModeSchedule.modeSequence.begin() - newLastActiveModeIndex - 1 > 0) {
      mismatchedIndex.first -= 1;
      mismatchedIndex.second -= 1;
    }
    // end TODO
    w = std::distance(oldModeSchedule.modeSequence.begin(), mismatchedIndex.first) - oldStartIndexOfMatchedSequence;

    if (w > 0) {
      break;
    } else {
      oldStartIndexOfMatchedSequence++;
    }
  }

  // matching event times.
  // note that (beginEventItr + w - 1) <= endEventItr. There are w - 1 event times between w modes.
  scalar_array_t oldMatchedEventTimes;
  scalar_array_t newMatchedEventTimes;
  if (w > 0) {
    const auto oldBeginEventItr = oldModeSchedule.eventTimes.begin() + oldStartIndexOfMatchedSequence;
    oldMatchedEventTimes.assign(oldBeginEventItr, oldBeginEventItr + w - 1);

    const auto newBeginEventItr = newModeSchedule.eventTimes.begin() + newStartIndexOfMatchedSequence;
    newMatchedEventTimes.assign(newBeginEventItr, newBeginEventItr + w - 1);
  }
  // either to add the event time which triggered the first matching mode or not
  // Case 2: The first matched event was inside the period but now it is outside
  if (oldStartIndexOfMatchedSequence > oldFirstActiveModeIndex) {
    const auto oldLastTriggeredEvent = oldModeSchedule.eventTimes[oldStartIndexOfMatchedSequence - 1];
    auto newLastTriggeredEvent = oldInitTime - 1e-2;  // a bit before the updated initial time

    oldMatchedEventTimes.insert(oldMatchedEventTimes.begin(), oldLastTriggeredEvent);
    newMatchedEventTimes.insert(newMatchedEventTimes.begin(), newLastTriggeredEvent);
  }

  bool isLastActiveModeOfOldModeSequenceMatched = oldStartIndexOfMatchedSequence + w - 1 == oldLastActiveModeIndex;
  bool isLastActiveModeOfNewModeSequenceMatched = newStartIndexOfMatchedSequence + w - 1 == newLastActiveModeIndex;
  if (!isLastActiveModeOfOldModeSequenceMatched &&
      (!isLastActiveModeOfNewModeSequenceMatched || oldModeSchedule.eventTimes[oldStartIndexOfMatchedSequence + w - 1] <
                                                        newModeSchedule.eventTimes[newStartIndexOfMatchedSequence + w - 1])) {
    const auto oldLEndEvent = oldModeSchedule.eventTimes[oldStartIndexOfMatchedSequence + w - 1];
    auto newEndEvent =
        isLastActiveModeOfNewModeSequenceMatched ? oldFinalTime + 1e-2 : newModeSchedule.eventTimes[newStartIndexOfMatchedSequence + w - 1];

    oldMatchedEventTimes.push_back(oldLEndEvent);
    newMatchedEventTimes.push_back(newEndEvent);
  }

  assert(oldMatchedEventTimes.size() == newMatchedEventTimes.size());

  // find the start index of erasing(include the start index) (by default preserve all)
  eraseFromIndex_ = oldTimeTrajectory.size();
  if (w == 0) {
    // erase all
    eraseFromIndex_ = 0;
  }
  // if last mode of the new mode sequence is NOT matched
  else if (!isLastActiveModeOfNewModeSequenceMatched) {
    // if both model subsequences match until the end of current mode but the updated one have more modes
    const auto mismatchEventTime = newModeSchedule.eventTimes[newStartIndexOfMatchedSequence + w - 1];
    eraseFromIndex_ = lowerBoundIndex(oldTimeTrajectory, mismatchEventTime);
  }

  // // computes the index of the spreading values and periods
  computeSpreadingStrategy(oldTimeTrajectory, oldMatchedEventTimes, newMatchedEventTimes);

  // debug print
  if (debugCaching_) {
    const auto oldFirstMatchedModeItr = oldModeSchedule.modeSequence.begin() + oldStartIndexOfMatchedSequence;
    const auto newFirstMatchedModeItr = newModeSchedule.modeSequence.begin() + newStartIndexOfMatchedSequence;

    std::cerr << "[TrajectorySpreading]:\n";
    std::cerr << "    ### Old Mode:         ";
    for (int i = 0; i < oldModeSchedule.modeSequence.size(); i++) {
      if (i == oldFirstActiveModeIndex) {
        std::cerr << "\033[0;33m";
      }

      std::cerr << oldModeSchedule.modeSequence[i];

      if (i != 0) {
        std::cerr << "(" << oldModeSchedule.eventTimes[i - 1] << ")";
      }

      if (i == oldFirstActiveModeIndex) {
        std::cerr << " \033[0;34m|" << oldInitTime << "|\033[0;33m";
      }

      if (i == oldLastActiveModeIndex) {
        std::cerr << " \033[0;34m|" << oldFinalTime << "|\033[0;33m";
      }
      std::cerr << " ";

      if (i == oldLastActiveModeIndex) {
        std::cerr << "\033[0m";
      }
    }
    std::cerr << "\n";

    std::cerr << "    ### New Mode:         ";
    for (int i = 0; i < newModeSchedule.modeSequence.size(); i++) {
      if (i == newFirstActiveModeIndex) {
        std::cerr << "\033[0;33m";
      }

      std::cerr << newModeSchedule.modeSequence[i];

      if (i != 0) {
        std::cerr << "(" << newModeSchedule.eventTimes[i - 1] << ")";
      }

      if (i == newFirstActiveModeIndex) {
        std::cerr << " \033[0;34m|" << oldInitTime << "|\033[0;33m";
      }
      if (i == newLastActiveModeIndex) {
        std::cerr << " \033[0;34m|" << oldFinalTime << "|\033[0;33m";
      }
      std::cerr << " ";

      if (i == newLastActiveModeIndex) {
        std::cerr << "\033[0m";
      }
    }
    std::cerr << "\n";

    std::cerr << "    ### Time Period:     [" << oldTimeTrajectory.front() << ", " << oldTimeTrajectory.back() << "]\n";
    std::cerr << "    ### Time Trajectory: {";
    for (int i = 0; i < oldTimeTrajectory.size(); i++) {
      if (i % 5 == 0) {
        std::cerr << "\033[0;33m|" << i << "|\033[0m";
      }
      std::cerr << oldTimeTrajectory[i] << "  ";
    }
    std::cerr << "}\n";
    std::cerr << "    ### Updated Mode Active At " << *newFirstMatchedModeItr << "\n";
    std::cerr << "    ### Matching Mode Window: " << w << "\n";

    std::cerr << "    ### Matching Mode:         ";
    std::for_each(oldFirstMatchedModeItr, oldFirstMatchedModeItr + w, [](size_t i) { std::cerr << i << "  "; });
    std::cerr << "\n";
    std::cerr << "    ### Matching Updated Mode: ";
    std::for_each(newFirstMatchedModeItr, newFirstMatchedModeItr + w, [](size_t i) { std::cerr << i << "  "; });
    std::cerr << "\n";

    std::cerr << "    ### Matching Event Times:         ";
    std::for_each(oldMatchedEventTimes.begin(), oldMatchedEventTimes.end(), [](scalar_t t) { std::cerr << t << "  "; });
    std::cerr << "\n";
    std::cerr << "    ### Matching Updated Event Times: ";
    std::for_each(newMatchedEventTimes.begin(), newMatchedEventTimes.end(), [](scalar_t t) { std::cerr << t << "  "; });
    std::cerr << "\n";
    std::cerr << "    ### Updated PostEventIndices: ";
    std::for_each(updatedPostEventIndices_.begin(), updatedPostEventIndices_.end(), [](size_t i) { std::cerr << i << "  "; });
    std::cerr << "\n";
    std::cerr << "    ### Updated Matched Times: ";
    std::for_each(updatedMatchedventTimes_.begin(), updatedMatchedventTimes_.end(), [](scalar_t t) { std::cerr << t << "  "; });

    if (eraseFromIndex_ == 0) {
      std::cerr << "    ### Will erase the whole trajectory!\n";
    } else if (eraseFromIndex_ < oldTimeTrajectory.size()) {
      std::cerr << "Spreading strategy [format: index] \n";
      std::cerr << "    ###  0  <-- keeping --> " << eraseFromIndex_ << " <-- erasing --> " << oldTimeTrajectory.size() - 1 << "\n";
      std::cerr << "Spreading strategy [format: time] \n";
      std::cerr << "    ### " << oldTimeTrajectory[0] << " <-- keeping --> " << oldTimeTrajectory[eraseFromIndex_] << " <-- erasing --> "
                << oldTimeTrajectory.back() << "\n";
    } else {
      std::cerr << "    ### Will not erase any part of the trajectory!\n";
    }

    std::cerr << "    ### trajectory spreading strategy for trajectories [format: [begin, end)] \n";
    for (size_t i = 0; i < spreadingValueIndices_.size(); i++) {
      std::cerr << "        " << beginIndices_[i] << " --> " << endIndices_[i] << " using value at " << spreadingValueIndices_[i] << "\n";
    }
  }
}  // namespace ocs2

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TrajectorySpreading::computeSpreadingStrategy(const scalar_array_t& oldTimeTrajectory, const scalar_array_t& oldMatchedEventTimes,
                                                   const scalar_array_t& newMatchedEventTimes) {
  // finds the indices of the event times
  size_array_t oldPostEventIndices = findPostEventIndices(oldMatchedEventTimes, oldTimeTrajectory);

  // finds the indices of the update event times
  size_array_t newPostEventIndices = findPostEventIndices(newMatchedEventTimes, oldTimeTrajectory);

  // spreading periods and value indices
  beginIndices_.clear();
  endIndices_.clear();
  spreadingValueIndices_.clear();

  // newPostEventIndices_.clear();
  // newMatchedventTimes_.clear();
  for (size_t j = 0; j < oldPostEventIndices.size(); j++) {
    const size_t oldPreEventIndex = oldPostEventIndices[j] - 1;

    // backward
    if (newPostEventIndices[j] < oldPostEventIndices[j]) {
      const size_t endIndex = std::min(oldPostEventIndices[j], eraseFromIndex_);
      beginIndices_.push_back(newPostEventIndices[j]);
      endIndices_.push_back(endIndex);
      spreadingValueIndices_.push_back(oldPostEventIndices[j]);

    }
    // forward
    else if (newPostEventIndices[j] > oldPostEventIndices[j]) {
      const size_t beginIndex = (j == 0 ? oldPostEventIndices[j] : std::max(oldPostEventIndices[j], newPostEventIndices[j - 1]));
      beginIndices_.push_back(beginIndex);
      endIndices_.push_back(newPostEventIndices[j]);
      spreadingValueIndices_.push_back(oldPreEventIndex);
    }

    if (newPostEventIndices[j] != 0 && newPostEventIndices[j] < eraseFromIndex_) {
      updatedPostEventIndices_.push_back(newPostEventIndices[j]);
      updatedMatchedventTimes_.push_back(newMatchedEventTimes[j]);
    }
  }  // end of j loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_array_t TrajectorySpreading::findPostEventIndices(const scalar_array_t& eventTimes, const scalar_array_t& timeTrajectory) const {
  size_array_t postEventIndices(eventTimes.size());
  std::transform(eventTimes.cbegin(), eventTimes.cend(), postEventIndices.begin(),
                 [this, &timeTrajectory](scalar_t time) -> int { return upperBoundIndex(timeTrajectory, time); });
  return postEventIndices;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TrajectorySpreading::adjustTimings(scalar_array_t& timeTrajectory, size_array_t& postEventIndices) const {
  timeTrajectory.erase(timeTrajectory.begin() + eraseFromIndex_, timeTrajectory.end());
  postEventIndices = updatedPostEventIndices_;

  for (size_t i = 0; i < updatedPostEventIndices_.size(); i++) {
    assert(updatedPostEventIndices_[i] < timeTrajectory.size());
    constexpr auto eps = numeric_traits::weakEpsilon<scalar_t>();
    timeTrajectory[updatedPostEventIndices_[i] - 1] = updatedMatchedventTimes_[i];
    timeTrajectory[updatedPostEventIndices_[i]] = std::min(updatedMatchedventTimes_[i] + eps, timeTrajectory.back());
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::stringstream& operator<<(std::stringstream& out, const std::pair<int, int>& ind) {
  out << "(" << ind.first << ", " << ind.second << ")";
  return out;
}

}  // namespace ocs2
