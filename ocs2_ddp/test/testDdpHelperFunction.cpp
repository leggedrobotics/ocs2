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

#include <iostream>

#include <gtest/gtest.h>

#include <ocs2_ddp/DDP_HelperFunctions.h>

using namespace ocs2;

TEST(extractPrimalSolution, eventAtInitTime) {
  constexpr size_t numTime = 5;
  constexpr int numSubsystem = 4;
  constexpr scalar_t initTime = 0.0;
  constexpr scalar_t timeStep = 0.2;
  constexpr auto eps = numeric_traits::weakEpsilon<scalar_t>();

  PrimalSolution primalSolution;
  for (int s = 0; s < numSubsystem - 1; ++s) {
    const auto t0 = (s == 0) ? initTime : primalSolution.timeTrajectory_.back();
    const auto t0Biased = (s == 0) ? t0 : (t0 + eps);

    for (size_t n = 0; n <= numTime; ++n) {
      primalSolution.timeTrajectory_.push_back((n == 0) ? t0Biased : t0 + n * timeStep);
      primalSolution.stateTrajectory_.push_back((vector_t(1) << 1.0 + s).finished());
      primalSolution.inputTrajectory_.push_back((vector_t(1) << -1.0 - s).finished());
    }  // end of n

    // there is an extra event at final time
    primalSolution.postEventIndices_.push_back(primalSolution.timeTrajectory_.size());
    primalSolution.modeSchedule_.modeSequence.push_back(s + 1);
    primalSolution.modeSchedule_.eventTimes.push_back(primalSolution.timeTrajectory_.back());
  }  // end of s

  // an extra event at final time
  primalSolution.timeTrajectory_.push_back(primalSolution.timeTrajectory_.back());
  primalSolution.stateTrajectory_.push_back((vector_t(1) << numSubsystem).finished());
  primalSolution.inputTrajectory_.push_back((vector_t(1) << -numSubsystem).finished());

  //  std::cerr << ">>>>>> original\n" << primalSolution << "\n";

  // test0: initial and final time events
  PrimalSolution PrimalSolutionTest0;
  const std::pair<scalar_t, scalar_t> timePeriodTest0{1.0, 2.0};
  extractPrimalSolution(timePeriodTest0, primalSolution, PrimalSolutionTest0);
  //  std::cerr << ">>>>>> Test 0\n" << PrimalSolutionTest0 << "\n";
  EXPECT_NEAR(PrimalSolutionTest0.timeTrajectory_.front(), timePeriodTest0.first, 1.5 * eps);
  EXPECT_NEAR(PrimalSolutionTest0.timeTrajectory_.back(), timePeriodTest0.second, 1e-8);
  EXPECT_EQ(PrimalSolutionTest0.postEventIndices_, size_array_t{PrimalSolutionTest0.timeTrajectory_.size() - 1});

  // test1: extracting the whole
  PrimalSolution PrimalSolutionTest1;
  const std::pair<scalar_t, scalar_t> timePeriodTest1{primalSolution.timeTrajectory_.front(), primalSolution.timeTrajectory_.back()};
  extractPrimalSolution(timePeriodTest1, primalSolution, PrimalSolutionTest1);
  //  std::cerr << ">>>>>> Test 1\n" << PrimalSolutionTest1 << "\n";
  EXPECT_EQ(primalSolution.timeTrajectory_, PrimalSolutionTest1.timeTrajectory_);
  EXPECT_EQ(primalSolution.postEventIndices_, PrimalSolutionTest1.postEventIndices_);

  // test2: extracting at non-event time
  PrimalSolution PrimalSolutionTest2;
  const std::pair<scalar_t, scalar_t> timePeriodTest2{1.2, 2.4};
  extractPrimalSolution(timePeriodTest2, primalSolution, PrimalSolutionTest2);
  //  std::cerr << ">>>>>> Test 2\n" << PrimalSolutionTest2 << "\n";
  EXPECT_EQ(PrimalSolutionTest2.timeTrajectory_.front(), timePeriodTest2.first);
  EXPECT_EQ(PrimalSolutionTest2.timeTrajectory_.back(), timePeriodTest2.second);
  EXPECT_EQ(PrimalSolutionTest2.postEventIndices_.size(), 1);

  // test3: extract zero length
  PrimalSolution PrimalSolutionTest3;
  const std::pair<scalar_t, scalar_t> timePeriodTest3{1.2, 1.2};
  extractPrimalSolution(timePeriodTest3, primalSolution, PrimalSolutionTest3);
  //  std::cerr << ">>>>>> Test 3\n" << PrimalSolutionTest3 << "\n";
  EXPECT_EQ(PrimalSolutionTest3.timeTrajectory_.size(), 1);
}
