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

#pragma once

#include <iostream>

#include <ocs2_core/Dimensions.h>

namespace ocs2 {

/**
 * This class contains the observation information.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SystemObservation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;

  /**
   * Constructor
   */
  SystemObservation() : subsystem_(0), time_(0.0), state_(state_vector_t::Zero()), input_(input_vector_t::Zero()) {}

  /**
   * Destructor
   */
  ~SystemObservation() = default;

  /**
   * Swap with other.
   * @param other
   */
  void swap(SystemObservation<STATE_DIM, INPUT_DIM>& other) {
    std::swap(time_, other.time_);
    std::swap(state_, other.state_);
    std::swap(input_, other.input_);
    std::swap(subsystem_, other.subsystem_);
  }

  inline scalar_t& time() { return time_; };
  inline const scalar_t& time() const { return time_; };

  inline state_vector_t& state() { return state_; };
  inline const state_vector_t& state() const { return state_; };
  inline scalar_t& state(const size_t& i) { return state_(i); };
  inline const scalar_t& state(const size_t& i) const { return state_(i); };

  inline input_vector_t& input() { return input_; };
  inline const input_vector_t& input() const { return input_; };
  inline scalar_t& input(const size_t& i) { return input_(i); };
  inline const scalar_t& input(const size_t& i) const { return input_(i); };

  inline size_t& subsystem() { return subsystem_; };
  inline const size_t& subsystem() const { return subsystem_; };

  inline void display() const {
    std::cerr << "Observation: " << std::endl;
    std::cerr << "\t time:      " << time_ << std::endl;
    std::cerr << "\t subsystem: " << subsystem_ << std::endl;
    std::cerr << "\t state:    [";
    for (int i = 0; i < state_.size() - 1; i++) {
      std::cerr << state_(i) << ", ";
    }
    std::cerr << state_(state_.size() - 1) << "]" << std::endl;
    std::cerr << "\t input:    [";
    for (int i = 0; i < input_.size() - 1; i++) {
      std::cerr << input_(i) << ", ";
    }
    std::cerr << input_(input_.size() - 1) << "]" << std::endl;
  }

 private:
  size_t subsystem_;
  scalar_t time_;
  state_vector_t state_;
  input_vector_t input_;
};

}  // namespace ocs2
