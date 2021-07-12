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

#include <ostream>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot {

/**
 * A gait is a periodic mode schedule parameterized by a "phase" variable.
 *
 * The eventPhases only indicate switches of modes, i.e. phase = 0 and phase = 1 are not part of the eventPhases.
 * The number of modes is therefore number of phases + 1
 *
 * The conversion to time is regulated by a duration
 */
struct Gait {
  /** time for one gait cycle*/
  scalar_t duration;
  /** points in (0.0, 1.0) along the gait cycle where the contact mode changes, size N-1 */
  std::vector<scalar_t> eventPhases;
  /** sequence of contact modes, size: N */
  std::vector<size_t> modeSequence;
};

/**
 * isValidGait checks the following properties
 * - positive duration
 * - eventPhases are all in (0.0, 1.0)
 * - eventPhases are sorted
 * - the size of the modeSequences is 1 more than the eventPhases.
 */
bool isValidGait(const Gait& gait);

/** Check is if the phase is in [0.0, 1.0) */
bool isValidPhase(scalar_t phase);

/** Wraps a phase to [0.0, 1.0) */
scalar_t wrapPhase(scalar_t phase);

/** The modes are selected with a closed-open interval: [ ) */
int getModeIndexFromPhase(scalar_t phase, const Gait& gait);

/** Gets the active mode from the phase variable */
size_t getModeFromPhase(scalar_t phase, const Gait& gait);

/** Returns the time left in the gait based on the phase variable */
scalar_t timeLeftInGait(scalar_t phase, const Gait& gait);

/** Returns the time left in the current based on the phase variable */
scalar_t timeLeftInMode(scalar_t phase, const Gait& gait);

/** Print gait */
std::ostream& operator<<(std::ostream& stream, const Gait& gait);

}  // namespace legged_robot
}  // namespace ocs2
