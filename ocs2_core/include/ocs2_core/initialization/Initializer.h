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

namespace ocs2 {

/**
 * This is the base class for initializing the solvers.
 */
class Initializer {
 public:
  /**
   * Constructor
   *
   * @param [in] inputDim: The dimension of input space.
   */
  explicit Initializer(int inputDim) : inputDim_(inputDim) {}

  /** Default destructor */
  virtual ~Initializer() = default;

  /**
   * Clones the class.
   *
   * @return A raw pointer to the class.
   */
  virtual Initializer* clone() const { return new Initializer(*this); }

  /**
   * Computes the state and input of the next time step based on the current time and state. Note that it guaranteed that there is
   * no event times in the interval [time, nextTime), but there might be an event at nextTime (with nextTime = time + timeStep).
   *
   * @param [in] time: The current time.
   * @param [in] state: The current state.
   * @param [in] nextTime: The next time step.
   * @param [out] input: The current input.
   * @param [out] nextState: The next state.
   */
  virtual void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
    input.setZero(inputDim_);
    nextState = state;
  }

 protected:
  /** Copy constructor */
  Initializer(const Initializer& rhs) = default;

  int inputDim_;
};

}  // namespace ocs2
