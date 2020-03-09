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

//
// Created by rgrandia on 09.03.20.
//

#pragma once

#include <string>
#include <vector>

#include <ocs2_core/Dimensions.h>

namespace ocs2 {

/**
 * Defines a sequence of N modes, separated by N-1 event times
 */
class ModeSchedule {
  using scalar_t = typename Dimensions<0, 0>::scalar_t;

 public:
  /**
   * Constructor for a modeschedule. The number of phases must be greater than zero (N > 0)
   * @param eventTimes : event times of size N - 1
   * @param modeSequence : mode sequence fo size N
   */
  ModeSchedule(std::vector<scalar_t> eventTimes = {}, std::vector<size_t> modeSequence = {0});

  /** Gets the event times */
  const std::vector<scalar_t>& eventTimes() const { return eventTimes_; }

  /** Gets the sequence of modes */
  const std::vector<size_t>& modeSequence() const { return modeSequence_; }

 private:
  std::vector<scalar_t> eventTimes_;
  std::vector<size_t> modeSequence_;
};

std::string display(const ModeSchedule& modeSchedule);

}