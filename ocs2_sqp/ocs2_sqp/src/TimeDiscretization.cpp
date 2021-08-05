/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "ocs2_sqp/TimeDiscretization.h"

#include <ocs2_core/misc/Lookup.h>

namespace ocs2 {

scalar_t getInterpolationTime(const AnnotatedTime& annotatedTime) {
  return annotatedTime.time + numeric_traits::limitEpsilon<scalar_t>();
}

scalar_t getIntervalStart(const AnnotatedTime& start) {
  scalar_t adaptedStart = start.time;
  if (start.event == AnnotatedTime::Event::PostEvent) {
    adaptedStart += numeric_traits::weakEpsilon<scalar_t>();
  }
  return adaptedStart;
}

scalar_t getIntervalEnd(const AnnotatedTime& end) {
  scalar_t adaptedEnd = end.time;
  if (end.event == AnnotatedTime::Event::PreEvent) {
    adaptedEnd -= numeric_traits::weakEpsilon<scalar_t>();
  }
  return adaptedEnd;
}

scalar_t getIntervalDuration(const AnnotatedTime& start, const AnnotatedTime& end) {
  return getIntervalEnd(end) - getIntervalStart(start);
}

std::vector<AnnotatedTime> timeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt,
                                                        const scalar_array_t& eventTimes, scalar_t dt_min) {
  assert(dt > 0);
  assert(finalTime > initTime);
  std::vector<AnnotatedTime> timeDiscretization;

  // Initialize
  timeDiscretization.emplace_back(initTime, AnnotatedTime::Event::None);
  scalar_t nextEventIdx = lookup::findIndexInTimeArray(eventTimes, initTime);

  // Fill iteratively with pre event, post events are added later
  AnnotatedTime nextNode = timeDiscretization.back();
  while (timeDiscretization.back().time < finalTime) {
    nextNode.time = nextNode.time + dt;
    nextNode.event = AnnotatedTime::Event::None;

    // Check if an event has passed
    if (nextEventIdx < eventTimes.size() && nextNode.time >= eventTimes[nextEventIdx]) {
      nextNode.time = eventTimes[nextEventIdx];
      nextNode.event = AnnotatedTime::Event::PreEvent;
      nextEventIdx++;
    }

    // Check if final time has passed
    if (nextNode.time >= finalTime) {
      nextNode.time = finalTime;
      nextNode.event = AnnotatedTime::Event::None;
    }

    if (nextNode.time > timeDiscretization.back().time + dt_min) {
      timeDiscretization.push_back(nextNode);
    } else {  // Points are close together -> overwrite the old point
      timeDiscretization.back() = nextNode;
    }
  }

  // Skip events at beginning of horizon
  if (timeDiscretization.front().event == AnnotatedTime::Event::PreEvent) {
    timeDiscretization.front().event = AnnotatedTime::Event::PostEvent;
  }

  // Duplicate all preEvents to postEvents
  std::vector<AnnotatedTime> timeDiscretizationWithDoubleEvents;
  timeDiscretizationWithDoubleEvents.reserve(2 * timeDiscretization.size());  // upper bound on size

  for (const auto& t : timeDiscretization) {
    timeDiscretizationWithDoubleEvents.push_back(t);
    if (t.event == AnnotatedTime::Event::PreEvent) {
      timeDiscretizationWithDoubleEvents.push_back(t);
      timeDiscretizationWithDoubleEvents.back().event = AnnotatedTime::Event::PostEvent;
    }
  }

  return timeDiscretizationWithDoubleEvents;
}

}  // namespace ocs2
