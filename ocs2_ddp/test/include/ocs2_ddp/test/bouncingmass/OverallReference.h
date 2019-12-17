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

#include "ocs2_ddp/test/bouncingmass/Reference.h"

/*
 * 	This class constructs, contains and provides the reference for the
 * 	BouncingMass SLQ test. The reference is based on the reference presented
 * 	in [On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints
 * by M. Rijnen]
 */
class OverallReference {
  using DIMENSIONS = ocs2::Dimensions<3, 1>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;

 public:
  /*
   * Constructor
   */
  OverallReference() = default;

  /*
   * Constructor
   *
   * @param [in] trajTimes: list of times at which the reference is defined
   * @param [in] trajState: list of waypoints at which the reference is defined
   */
  OverallReference(const scalar_array_t trajTimes, const state_vector_array_t trajState);

  /*
   * Calculate the input at a certain time
   *
   * @param [in] time: time moment at which the input is calculated
   * @param [out] input: input corresponding to time
   */
  void getInput(scalar_t time, input_vector_t& input);

  /*
   * Calculate the input at a certain time
   *
   * @param [in] time: time moment at which the input is calculated
   * @return input corresponding to time
   */
  input_vector_t getInput(scalar_t time);

  /*
   * Calculate the reference state at a certain time
   *
   * @param [in] time: time moment at which the input is calculated
   * @param [out] state: state corresponding to time
   */
  void getState(scalar_t time, state_vector_t& x);

  /*
   * Calculate the reference state at a certain time
   *
   * @param [in] time: time moment at which the input is calculated
   * @return state corresponding to time
   */
  state_vector_t getState(scalar_t time);

  /*
   * Calculate the reference state at a certain time and mode
   *
   * @param [in] idx: mode at which the input is calculated
   * @param [in] time: time moment at which the input is calculated
   * @return state corresponding to time and mode
   */
  state_vector_t getState(int idx, scalar_t time);

  /*
   * Calculate the reference state at a certain time and mode
   *
   * @param [in] idx: mode at which the input is calculated
   * @param [in] time: time moment at which the input is calculated
   * @param [out] state: state corresponding to time and mode
   */
  void getState(int idx, scalar_t time, state_vector_t& x);

  /*
   * Extend the reference past the event times, by integrating the input signal
   * without applying the jump map
   *
   * @param [in] delta: length of the extensions
   */
  void extendref(scalar_t delta);

  /*
   * Display the reference
   *
   * @param [in] i: choice of which section of the reference is displayed
   */
  void display(int i);

 private:
  /*
   * Find the index of the currently active reference
   *
   * @param [in] time: time moment at which the index is requested
   *
   * @return currently active index
   */
  int getIndex(scalar_t time);

  /*
   * Jump map of the system
   *
   * @param [in] x: State before event
   *
   * @return currently active index
   */
  void jumpMap(state_vector_t& x);

  std::vector<Reference> References_;
  std::vector<scalar_t> switchtimes_;
};
