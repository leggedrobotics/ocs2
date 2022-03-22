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

#include "ocs2_oc/trajectory_adjustment/TrajectorySpreading.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>

#include <ocs2_core/NumericTraits.h>
#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

namespace {
std::ostream& operator<<(std::ostream& os, const std::pair<int, int>& ind) {
  os << "(" << ind.first << ", " << ind.second << ")";
  return os;
}

/**
 * Returns the first mismatching pair of elements from two ranges: one defined by [first1, last1) and
 * another defined by [first2,last2).
 *
 * TODO deprecate this function once switched to c++14 and use: std::mismatch(first1, last1, first2, last2)
 */
template <class InputIt1, class InputIt2>
std::pair<InputIt1, InputIt2> mismatch(InputIt1 first1, InputIt1 last1, InputIt2 first2, InputIt2 last2) {
  auto mismatchedIndex = std::mismatch(first1, last1, first2);
  while (std::distance(last2, mismatchedIndex.second) > 0) {
    --mismatchedIndex.first;
    --mismatchedIndex.second;
  }
  return mismatchedIndex;
}

}  // anonymous namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto TrajectorySpreading::set(const ModeSchedule& oldModeSchedule, const ModeSchedule& newModeSchedule,
                              const scalar_array_t& oldTimeTrajectory) -> Status {
  // step 1: What modes do the original primal solution contain?
  const scalar_t oldInitTime = oldTimeTrajectory.front();
  const scalar_t oldFinalTime = oldTimeTrajectory.back();

  // note: sizeof(eventTimes) + 1 == sizeof(modeSequence)
  const int oldFirstActiveModeIndex = upperBoundIndex(oldModeSchedule.eventTimes, oldInitTime);  // no event at initial time
  const int oldLastActiveModeIndex = upperBoundIndex(oldModeSchedule.eventTimes, oldFinalTime);  // can be an event at final time

  // step 2: What modes do the new mode schedule need?
  const int newFirstActiveModeIndex = upperBoundIndex(newModeSchedule.eventTimes, oldInitTime);  // no event at initial time
  const int newLastActiveModeIndex = upperBoundIndex(newModeSchedule.eventTimes, oldFinalTime);  // can be an event at final time

  // the starting mode of the matched sequence may differ from the leading mode of the old mode schedule.
  auto oldStartIndexOfMatchedSequence = oldFirstActiveModeIndex;
  // the starting mode of the matched sequence must cover the newInitTime
  const auto newStartIndexOfMatchedSequence = newFirstActiveModeIndex;

  // step 3: What is the longest window of matched mode sequence?
  // the size of the window where the active modes are identical.
  // this means: mode[firstMatchingModeIndex + w] != updatedMode[updatedFirstMatchingModeIndex + w]
  size_t w = 0;
  while (oldStartIndexOfMatchedSequence < oldModeSchedule.modeSequence.size()) {
    // TODO: change to std::mismatch(first1, last1, first2, last2). It is supported since c++14
    // +1 to include the last active mode
    const auto mismatchedIndex = mismatch(oldModeSchedule.modeSequence.cbegin() + oldStartIndexOfMatchedSequence,
                                          oldModeSchedule.modeSequence.cbegin() + oldLastActiveModeIndex + 1,
                                          newModeSchedule.modeSequence.cbegin() + newStartIndexOfMatchedSequence,
                                          newModeSchedule.modeSequence.cbegin() + newLastActiveModeIndex + 1);

    w = std::distance(oldModeSchedule.modeSequence.begin() + oldStartIndexOfMatchedSequence, mismatchedIndex.first);

    if (w > 0) {
      break;
    } else {
      // move starting index forward to find the first matched mode in the old mode schedule
      ++oldStartIndexOfMatchedSequence;
    }
  }

  // matched event times. Used to store event time that is likely to be spread.
  // note: (beginEventItr + w - 1) <= endEventItr. There are w - 1 event times between w modes.
  scalar_array_t oldMatchedEventTimes;
  scalar_array_t newMatchedEventTimes;
  keepEventDataInInterval_ = {0, 0};
  // Add corresponding matched event time
  // Phase 1: Add all event time that are between modes. Those event time show up in both original and modified trajectories.
  if (w > 0) {
    const auto oldBeginEventItr = oldModeSchedule.eventTimes.begin() + oldStartIndexOfMatchedSequence;
    oldMatchedEventTimes.assign(oldBeginEventItr, oldBeginEventItr + w - 1);

    const auto newBeginEventItr = newModeSchedule.eventTimes.begin() + newStartIndexOfMatchedSequence;
    newMatchedEventTimes.assign(newBeginEventItr, newBeginEventItr + w - 1);

    // w - 1 events are kept in total
    keepEventDataInInterval_.first = oldStartIndexOfMatchedSequence - oldFirstActiveModeIndex;
    keepEventDataInInterval_.second = keepEventDataInInterval_.first + w - 1;
  }

  // Phase 2: Add triggering event time if the first matched mode is NOT the leading mode of the old mode schedule.
  // As the first matched mode must be the leading mode of the new mode schedule, in the case that the first matched mode is NOT the
  // leading mode of the old mode schedule, the triggering event time need to be shifted back to the beginning of trajectories
  // In summary, Triggering event time can be either ignored (leading mode of both old and new schedule are matched) or shifted backward.
  if (w > 0 && oldStartIndexOfMatchedSequence > oldFirstActiveModeIndex) {
    const auto oldLastTriggeredEvent = oldModeSchedule.eventTimes[oldStartIndexOfMatchedSequence - 1];
    const auto newLastTriggeredEvent = oldInitTime - 1e-4;  // a bit before the updated initial time

    oldMatchedEventTimes.insert(oldMatchedEventTimes.begin(), oldLastTriggeredEvent);
    newMatchedEventTimes.insert(newMatchedEventTimes.begin(), newLastTriggeredEvent);
  }

  // Phase 3: Add ending event time if that of the old mode schedule is less than that of the new mode schedule.
  // The situation is similar to phase 2 where ending event time is added if it has to be spread till the end of truncated trajectories.
  // If the last active mode of the old mode schedule is matched, the ending event time can be either left untouched or moved backward(no
  // space for spreading forward). But in both cases, we don't have to add the ending event time as everything after the new ending event
  // time will be erased.
  // If the last active mode of the old mode schedule is NOT matched but the last active mode of the new mode schedule is matched, that
  // indicates the old ending event time need to be spread till the end of trajectories.
  // If neither new last active mode and old last active mode is matched, we have to compare the corresponding ending event time.
  const bool isLastActiveModeOfOldModeSequenceMatched = oldStartIndexOfMatchedSequence + w - 1 == oldLastActiveModeIndex;
  const bool isLastActiveModeOfNewModeSequenceMatched = newStartIndexOfMatchedSequence + w - 1 == newLastActiveModeIndex;
  if (!isLastActiveModeOfOldModeSequenceMatched &&
      (isLastActiveModeOfNewModeSequenceMatched || oldModeSchedule.eventTimes[oldStartIndexOfMatchedSequence + w - 1] <
                                                       newModeSchedule.eventTimes[newStartIndexOfMatchedSequence + w - 1])) {
    const auto oldLEndEvent = oldModeSchedule.eventTimes[oldStartIndexOfMatchedSequence + w - 1];
    const auto newEndEvent =
        isLastActiveModeOfNewModeSequenceMatched ? oldFinalTime + 1e-4 : newModeSchedule.eventTimes[newStartIndexOfMatchedSequence + w - 1];

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
    const auto mismatchEventTime = newModeSchedule.eventTimes[newStartIndexOfMatchedSequence + w - 1];
    eraseFromIndex_ = upperBoundIndex(oldTimeTrajectory, mismatchEventTime);
  }

  // computes the index of the spreading values and intervals
  computeSpreadingStrategy(oldTimeTrajectory, oldMatchedEventTimes, newMatchedEventTimes);

  // status
  const auto reportStatus = [&]() -> Status {
    Status status;
    status.willTruncate = (eraseFromIndex_ < oldTimeTrajectory.size());
    status.willPerformTrajectorySpreading = !spreadingValueIndices_.empty();
    return status;
  };

  // debug print
  if (debugPrint_) {
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
    std::for_each(updatedMatchedEventTimes_.begin(), updatedMatchedEventTimes_.end(), [](scalar_t t) { std::cerr << t << "  "; });

    if (eraseFromIndex_ == 0) {
      std::cerr << "    ### Will erase the whole trajectory!\n";
    } else if (eraseFromIndex_ < oldTimeTrajectory.size()) {
      std::cerr << "Spreading strategy [format: index] \n";
      std::cerr << "    ###  0  <-- keeping --> " << eraseFromIndex_ << " <-- erasing --> " << oldTimeTrajectory.size() - 1 << "\n";
      std::cerr << "Spreading strategy [format: time] \n";
      std::cerr << "    ### " << oldTimeTrajectory.front() << " <-- keeping --> " << oldTimeTrajectory[eraseFromIndex_]
                << " <-- erasing --> " << oldTimeTrajectory.back() << "\n";
    } else {
      std::cerr << "    ### Will not erase any part of the trajectory!\n";
    }

    std::cerr << "    ### Keep event data within [" << keepEventDataInInterval_.first << ", " << keepEventDataInInterval_.second << ")\n";

    std::cerr << "    ### trajectory spreading strategy for trajectories [format: [begin, end)] \n";
    for (size_t i = 0; i < spreadingValueIndices_.size(); i++) {
      std::cerr << "        " << beginIndices_[i] << " --> " << endIndices_[i] << " using value at " << spreadingValueIndices_[i] << "\n";
    }
  }

  return reportStatus();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TrajectorySpreading::computeSpreadingStrategy(const scalar_array_t& oldTimeTrajectory, const scalar_array_t& oldMatchedEventTimes,
                                                   const scalar_array_t& newMatchedEventTimes) {
  // output containers
  beginIndices_.clear();
  endIndices_.clear();
  spreadingValueIndices_.clear();
  updatedPostEventIndices_.clear();
  updatedMatchedEventTimes_.clear();

  // finds the indices of the post event time
  size_array_t oldPostEventIndices = findPostEventIndices(oldMatchedEventTimes, oldTimeTrajectory);

  // finds the indices of the new post event time
  size_array_t newPostEventIndices = findPostEventIndices(newMatchedEventTimes, oldTimeTrajectory);

  // spreading periods and value indices
  for (size_t j = 0; j < oldPostEventIndices.size(); j++) {
    const size_t oldPreEventIndex = oldPostEventIndices[j] - 1;

    /**
     * Backward spreading
     *
     * |- - - - - - - - * - - - - - -|  Old event
     *           _______|
     *          /
     * |- -  - * - |(erase from here)| New event
     *
     *
     * The end index should be smaller or equal to the index of the first erased time stamp
     */
    if (newPostEventIndices[j] < oldPostEventIndices[j]) {
      const size_t endIndex = std::min(oldPostEventIndices[j], eraseFromIndex_);
      beginIndices_.push_back(newPostEventIndices[j]);
      endIndices_.push_back(endIndex);
      spreadingValueIndices_.push_back(oldPostEventIndices[j]);

      // Check the assumption that zero length adjustment is excluded. Note that (newPostEventIndices[j] != eraseFromIndex_) since
      // eraseFromIndex_ will be the next postEventIndex after newPostEventIndices[j]. See phase 3 of adding matched event time.
      assert(beginIndices_.back() != endIndices_.back());
    }

    /**
     * Forward spreading
     *
     * |- * - - (*) - - * - - - -|  Old event
     *    |      |__________
     *    |_________        |
     *              |       |
     * |- - - - - - * - - -(*) - - | New event
     *
     *
     * The begin index should be larger than the previous new event index when overlapping occurs in the forward spreadind case. Otherwise
     * the current mode will override the previous mode resulting in wrong mode sequence.
     */
    else if (newPostEventIndices[j] > oldPostEventIndices[j]) {
      const size_t beginIndex = (j == 0 ? oldPostEventIndices[j] : std::max(oldPostEventIndices[j], newPostEventIndices[j - 1]));
      beginIndices_.push_back(beginIndex);
      endIndices_.push_back(newPostEventIndices[j]);
      spreadingValueIndices_.push_back(oldPreEventIndex);

      // Check the assumption that zero length adjustment is excluded. Note that oldPostEventIndices[j] and newPostEventIndices[j - 1]
      // must be different.
      assert(beginIndices_.back() != endIndices_.back());
    }

    // the first index can be post event index but it is meaningless to store it in the indices array.
    // similarly, the last index can be pre-event index but it is meaningless to store it in the indices array.
    if (newPostEventIndices[j] != 0 && newPostEventIndices[j] < eraseFromIndex_) {
      updatedPostEventIndices_.push_back(newPostEventIndices[j]);
      updatedMatchedEventTimes_.push_back(newMatchedEventTimes[j]);
    }
  }  // end of j loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TrajectorySpreading::adjustTimeTrajectory(scalar_array_t& timeTrajectory) const {
  timeTrajectory.erase(timeTrajectory.begin() + eraseFromIndex_, timeTrajectory.end());

  for (size_t i = 0; i < updatedPostEventIndices_.size(); i++) {
    assert(updatedPostEventIndices_[i] < timeTrajectory.size());
    constexpr auto eps = numeric_traits::weakEpsilon<scalar_t>();
    timeTrajectory[updatedPostEventIndices_[i] - 1] = updatedMatchedEventTimes_[i];
    timeTrajectory[updatedPostEventIndices_[i]] = std::min(updatedMatchedEventTimes_[i] + eps, timeTrajectory.back());
  }  // end of i loop
}

}  // namespace ocs2
