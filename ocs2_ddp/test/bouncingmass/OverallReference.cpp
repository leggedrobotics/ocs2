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

#include "ocs2_ddp/test/bouncingmass/OverallReference.h"

#include <ocs2_core/misc/Lookup.h>

OverallReference::OverallReference(const scalar_array_t& trajTimes, const vector_array_t& trajStates) {
  switchtimes_ = trajTimes;

  References_.reserve(trajTimes.size() - 1);
  for (int i = 0; i < trajTimes.size() - 1; i++) {
    if (i == 0) {
      References_.emplace_back(trajTimes[i], trajTimes[i + 1], trajStates[i], trajStates[i + 1]);
    } else {
      References_.emplace_back(trajTimes[i], trajTimes[i + 1], jumpMap(trajStates[i]), trajStates[i + 1]);
    }
  }
}

vector_t OverallReference::getInput(scalar_t time) const {
  const int idx = ocs2::lookup::findIntervalInTimeArray(switchtimes_, time);
  return (idx >= 0 && idx < References_.size()) ? References_[idx].getInput(time) : vector_t::Zero(INPUT_DIM);
}

vector_t OverallReference::getState(scalar_t time) const {
  const int idx = ocs2::lookup::findIntervalInTimeArray(switchtimes_, time);
  return (idx >= 0 && idx < References_.size()) ? References_[idx].getState(time) : References_[0].getState(0);
}

vector_t OverallReference::getState(int idx, scalar_t time) const {
  vector_t state;
  if (idx >= 0 && idx < References_.size()) {
    if (time < 2) {
      state = References_[idx].getState(time);
    } else {
      state = getState(time);
    }
  } else {
    state = References_[0].getState(0);
  }
  return state;
}

void OverallReference::extendref(scalar_t delta) {
  for (int i = 0; i < References_.size(); i++) {
    Reference* pre = i > 0 ? &References_[i - 1] : nullptr;
    Reference* post = i < References_.size() - 1 ? &References_[i + 1] : nullptr;
    References_[i].extendref(delta, pre, post);
  }
}

void OverallReference::display(int i) {
  References_[i].display();
}

vector_t OverallReference::jumpMap(const vector_t& x) const {
  constexpr scalar_t e = 0.95;
  vector_t y(x.size());
  y << x(0), -e * x(1), x(2) + 1.0;
  return y;
}
