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

#include <ocs2_core/OCS2NumericTraits.h>
#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Packs together a time, and if an event happens at exactly that time.
 */
struct AnnotatedTime {
  scalar_t time;
  bool isEvent;
};

/**
 * Computes the time at which to interpolate, respecting interpolation rules around event times
 *
 * @param time : Current time
 * @param end : End time of the interval
 * @return time to make the interpolation at.
 */
scalar_t getInterpolationTime(const AnnotatedTime& annotatedTime);

/**
 * Computes the interval start that respects interpolation rules around event times
 *
 * @param start : Time and event information at the start of the interval
 * @return adapted start time t
 */
scalar_t getIntervalStart(const AnnotatedTime& start);

/**
 * Computes the interval end that respects interpolation rules around event times
 *
 * @param end : Time and event information at the end of the interval
 * @return adapted end time t
 */
scalar_t getIntervalEnd(const AnnotatedTime& end);

/**
 * Computes the interval duration that respects interpolation rules around event times
 *
 * @param start : Time and event information at the start of the interval
 * @param end : Time and event information at the end of the interval
 * @return dt
 */
scalar_t getIntervalDuration(const AnnotatedTime& start, const AnnotatedTime& end);

/**
 * Decides on time discretization along the horizon. Tries to makes step of dt, but will also ensure that eventtimes are part of the
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
                                                        scalar_t dt_min = 10.0 * OCS2NumericTraits<scalar_t>::limitEpsilon());

}  // namespace ocs2