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

#include <chrono>
#include <thread>

namespace ocs2 {

/**
 * Helper function to execute a given function and sleep for the remainder of the specified timing.
 * Will not interrupt the function if it is too slow.
 *
 * @tparam Functor : function type
 * @param f : callable object to execute
 * @param frequency : the frequency the function should run at.
 */
template <typename Functor>
void executeAndSleep(Functor f, double frequency) {
  using clock = std::chrono::high_resolution_clock;
  const auto start = clock::now();

  // Execute wrapped function
  f();

  // Compute desired duration rounded to clock decimation
  const std::chrono::duration<double> desiredDuration(1.0 / frequency);
  const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

  // Sleep
  const auto sleepTill = start + dt;
  std::this_thread::sleep_until(sleepTill);
}

/**
 * Helper function to execute a given function at a given rate while a condition is true.
 * Will not interrupt the function if it is too slow.
 *
 * @tparam Functor : function type
 * @param f : callable object to execute.
 * @param condition : condition checked every loop, will exit when this function returns false.
 * @param frequency : the frequency the function should run at.
 */
template <typename Functor1, typename Functor2>
void executeAtRate(Functor1 f, Functor2 condition, double frequency) {
  using clock = std::chrono::high_resolution_clock;

  // Compute desired duration rounded to clock decimation
  const std::chrono::duration<double> desiredDuration(1.0 / frequency);
  const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

  // Initialize timing
  const auto start = clock::now();
  auto sleepTill = start + dt;

  // Execution loop
  while (condition()) {
    // Execute wrapped function
    f();

    // Sleep
    std::this_thread::sleep_until(sleepTill);
    sleepTill += dt;
  }
}

}  // namespace ocs2