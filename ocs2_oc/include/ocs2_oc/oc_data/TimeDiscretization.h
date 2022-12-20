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

#pragma once

#include <ocs2_core/NumericTraits.h>
#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Packs together a time, and if an event happens at exactly that time.
 */
struct AnnotatedTime {
  /// Enum defines the type of event that occurs at this time
  enum class Event { None, PreEvent, PostEvent };

  scalar_t time;
  Event event;

  /** Constructor defaulting to 'None' event  */
  explicit AnnotatedTime(scalar_t t, Event e = Event::None) : time(t), event(e){};
};

/** Computes the time at which to interpolate, respecting interpolation rules around event times */
scalar_t getInterpolationTime(const AnnotatedTime& annotatedTime);

/** Computes the interval start that respects interpolation rules around event times */
scalar_t getIntervalStart(const AnnotatedTime& start);

/** Computes the interval end that respects interpolation rules around event times */
scalar_t getIntervalEnd(const AnnotatedTime& end);

/** Computes the interval duration that respects interpolation rules around event times */
scalar_t getIntervalDuration(const AnnotatedTime& start, const AnnotatedTime& end);

/**
 * Decides on time discretization along the horizon. Tries to makes step of dt, but will also ensure that event times are part of the
 * discretization.
 *
 * @param initTime : start time.
 * @param finalTime : final time.
 * @param dt : desired discretization step.
 * @param eventTimes : Event times where a time discretization must be made.
 * @param dt_min : minimum discretization step. Smaller intervals will be merged. Needs to be bigger than limitEpsilon to avoid
 * interpolation problems
 * @return vector of discrete time points
 */
std::vector<AnnotatedTime> timeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt,
                                                        const scalar_array_t& eventTimes,
                                                        scalar_t dt_min = 10.0 * numeric_traits::limitEpsilon<scalar_t>());

/**
 * Extracts the time trajectory from the annotated time trajectory.
 *
 * @param annotatedTime : Annotated time trajectory.
 * @return The time trajectory.
 */
scalar_array_t toTime(const std::vector<AnnotatedTime>& annotatedTime);

/**
 * Extracts the time trajectory from the annotated time trajectory respecting interpolation rules around event times.
 *
 * @param annotatedTime : Annotated time trajectory.
 * @return The time trajectory.
 */
scalar_array_t toInterpolationTime(const std::vector<AnnotatedTime>& annotatedTime);

/**
 * Extracts the array of indices indicating the post-event times from the annotated time trajectory.
 *
 * @param annotatedTime : Annotated time trajectory.
 * @return The post-event indices.
 */
size_array_t toPostEventIndices(const std::vector<AnnotatedTime>& annotatedTime);

}  // namespace ocs2
