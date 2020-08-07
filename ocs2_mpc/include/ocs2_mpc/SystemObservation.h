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

#include <ocs2_core/Types.h>
#include <iostream>

namespace ocs2 {

// TODO(mspieler): change to sruct, define swap helper, delete default constructor.

/**
 * This class contains the observation information.
 */
class SystemObservation {
 public:
  /** Constructor */
  SystemObservation() = default;

  /** Destructor */
  ~SystemObservation() = default;

  /** Swap with other. */
  void swap(SystemObservation& other) {
    std::swap(time_, other.time_);
    std::swap(state_, other.state_);
    std::swap(input_, other.input_);
    std::swap(subsystem_, other.subsystem_);
  }

  inline scalar_t& time() { return time_; };
  inline const scalar_t& time() const { return time_; };

  inline vector_t& state() { return state_; };
  inline const vector_t& state() const { return state_; };
  inline scalar_t& state(const size_t& i) { return state_(i); };
  inline const scalar_t& state(const size_t& i) const { return state_(i); };

  inline vector_t& input() { return input_; };
  inline const vector_t& input() const { return input_; };
  inline scalar_t& input(const size_t& i) { return input_(i); };
  inline const scalar_t& input(const size_t& i) const { return input_(i); };

  inline size_t& subsystem() { return subsystem_; };
  inline const size_t& subsystem() const { return subsystem_; };

  inline void display() const {
    std::cerr << "Observation: \n";
    std::cerr << "\t time:      " << time_ << '\n';
    std::cerr << "\t subsystem: " << subsystem_ << '\n';
    std::cerr << "\t state:    [";
    for (int i = 0; i < state_.size() - 1; i++) {
      std::cerr << state_(i) << ", ";
    }
    std::cerr << state_(state_.size() - 1) << "]\n";
    std::cerr << "\t input:    [";
    for (int i = 0; i < input_.size() - 1; i++) {
      std::cerr << input_(i) << ", ";
    }
    std::cerr << input_(input_.size() - 1) << "]";
  }

 private:
  size_t subsystem_ = 0;
  scalar_t time_ = 0.0;
  vector_t state_ = vector_t();
  vector_t input_ = vector_t();
};

}  // namespace ocs2
