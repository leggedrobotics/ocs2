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
#include <utility>
#include <vector>

#include <ocs2_core/Dimensions.h>

namespace ocs2 {

/**
 * This class is ...
 */
class RootFind {
 public:
  using scalar_t = Dimensions<0, 0>::scalar_t;
  using interval_t = std::pair<scalar_t, scalar_t>;

  /**
   * Default Constructor.
   */
  RootFind() = default;

  /**
   * Default destructor.
   */
  ~RootFind() = default;

  /**
   * Set the initial bracket when the RootFinding method is initialized.
   * Normally done when first zero crossing is detected
   *
   * @param [in] time_int: Pair of two time moments
   * @param [in] guard_int: Pair of two function values at time_int times, should have opposite sign
   */
  void set_Init_Bracket(const interval_t& time_int, const interval_t& guard_int) {
    if (guard_int.first * guard_int.second > 0) {
      throw std::runtime_error("Bracket function values should have opposite sign");
    }

    time_int_m = time_int;
    guard_int_m = guard_int;
  }

  /**
   * Set the initial bracket when the RootFinding method is initialized.
   * Normally done when first zero crossing is detected.
   *
   * @param [in] t0: First time of bracketing interval.
   * @param [in] t1: Second time of bracketing interval.
   * @param [in] f0: Function value corresponding to t0.
   * @param [in] f1: Function value corresponding to t1, of opposite sign to f0.
   */
  void set_Init_Bracket(const scalar_t t0, const scalar_t t1, const scalar_t f0, const scalar_t f1) {
    if (f0 * f1 > 0) {
      std::cout << t0 << ";" << t1 << std::endl;
      std::cout << f0 << ";" << f1 << std::endl;
      throw std::runtime_error("Bracket function values should have opposite sign");
    }

    time_int_m = std::make_pair(t0, t1);
    guard_int_m = std::make_pair(f0, f1);
  }

  /**
   * Update Current bracket, based on based on sign of the new query point
   *
   * @param [in] query: Time moment of last query point
   * @param [in] f_query: Function evaluation of last query time
   */
  void Update_Bracket(const scalar_t& query, const scalar_t& f_query) {
    if (f_query * guard_int_m.first < 0) {
      guard_int_m.second = guard_int_m.first;
      time_int_m.second = time_int_m.first;

      guard_int_m.first = f_query;
      time_int_m.first = query;

    } else {
      scalar_t gamma = 1 - (f_query / guard_int_m.first);

      if (gamma < 0) {
        gamma = 0.5;
      }

      // gamma = guard_int_m.first/(guard_int_m.first + f_query); // Pegasus Method
      // gamma = 0.5;		// Illinois Method
      // gamma = 1;		// Regular Regula Falsi

      guard_int_m.first = f_query;
      time_int_m.first = query;

      guard_int_m.second *= gamma;
    }
  }

  /**
   * Use (adapted-) regula falsi method to obtain a new query point
   *
   * @param [out] query: Time moment of new query point
   *
   */
  void getNewQuery(double& query) {
    scalar_t fa = guard_int_m.first;
    scalar_t ta = time_int_m.first;

    scalar_t fb = guard_int_m.second;
    scalar_t tb = time_int_m.second;

    query = (ta * fb - tb * fa) / (fb - fa);
  }

  /**
   * Display relevant bracketing information
   *
   */
  void Display() {
    std::cout << "Root Finding Information" << std::endl;
    std::cout << "Time Bracket: [" << time_int_m.first << ";" << time_int_m.second << "]" << std::endl;
    std::cout << "Value Bracket: [" << guard_int_m.first << ";" << guard_int_m.second << "]" << std::endl;
  }

 private:
  interval_t time_int_m;
  interval_t guard_int_m;
};

}  // namespace ocs2
