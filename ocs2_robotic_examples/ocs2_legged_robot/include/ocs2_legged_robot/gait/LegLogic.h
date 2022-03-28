/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/reference/ModeSchedule.h>

#include "ocs2_legged_robot/common/Types.h"

namespace ocs2 {
namespace legged_robot {

struct LegPhase {
  scalar_t phase;
  scalar_t duration;
};

struct ContactTiming {
  scalar_t start;
  scalar_t end;
};

struct SwingTiming {
  scalar_t start;
  scalar_t end;
};

/**
 * @brief Get the contact phase for all legs.
 * If leg in contact, returns a value between 0.0 (at start of contact phase) and 1.0 (at end of contact phase).
 * If leg not in contact (i.e. in swing), returns -1.0.
 * If mode schedule starts with contact phase, returns 1.0 during this phase.
 * If mode schedule ends with contact phase, returns 0.0 during this phase.
 * @param [in] time : Query time.
 * @param [in] modeSchedule : Mode schedule.
 * @return Contact phases for all legs.
 */
feet_array_t<LegPhase> getContactPhasePerLeg(scalar_t time, const ocs2::ModeSchedule& modeSchedule);

/**
 * @brief Get the swing phase for all legs.
 * If leg in swing, returns a value between 0.0 (at start of swing phase) and 1.0 (at end of swing phase).
 * If leg not in swing (i.e. in contact), returns -1.0.
 * If mode schedule starts with swing phase, returns 1.0 during this phase.
 * If mode schedule ends with swing phase, returns 0.0 during this phase.
 * @param [in] time : Query time.
 * @param [in] modeSchedule : Mode schedule.
 * @return Swing phases for all legs.
 */
feet_array_t<LegPhase> getSwingPhasePerLeg(scalar_t time, const ocs2::ModeSchedule& modeSchedule);

/** Extracts the contact timings for all legs from a modeSchedule */
feet_array_t<std::vector<ContactTiming>> extractContactTimingsPerLeg(const ocs2::ModeSchedule& modeSchedule);

/** Extracts the swing timings for all legs from a modeSchedule */
feet_array_t<std::vector<SwingTiming>> extractSwingTimingsPerLeg(const ocs2::ModeSchedule& modeSchedule);

/** Returns time of the next lift off. Returns nan if leg is not lifting off */
scalar_t getTimeOfNextLiftOff(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

/** Returns time of the  touch down for all legs from a modeschedule. Returns nan if leg does not touch down */
scalar_t getTimeOfNextTouchDown(scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

/**
 * Get {startTime, endTime} for all contact phases. Swingphases are always implied in between: endTime[i] < startTime[i+1]
 * times are NaN if they cannot be identified at the boundaries
 * Vector is empty if there are no contact phases
 */
std::vector<ContactTiming> extractContactTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);

/**
 * Get {startTime, endTime} for all swing phases. Contact phases are always implied in between: endTime[i] < startTime[i+1]
 * times are NaN if they cannot be identified at the boundaries
 * Vector is empty if there are no swing phases
 */
std::vector<SwingTiming> extractSwingTimings(const std::vector<scalar_t>& eventTimes, const std::vector<bool>& contactFlags);

/**
 * Extracts for each leg the contact sequence over the motion phase sequence.
 * @param modeSequence : Sequence of contact modes.
 * @return Sequence of contact flags per leg.
 */
feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& modeSequence);

}  // namespace legged_robot
}  // namespace ocs2
