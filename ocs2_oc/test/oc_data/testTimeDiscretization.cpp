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

#include <gtest/gtest.h>

#include "ocs2_oc/oc_data/TimeDiscretization.h"

using namespace ocs2;

TEST(test_time_discretization, noEvents_plusEps) {
  scalar_t initTime = 0.1;
  scalar_t finalTime = 0.3 + std::numeric_limits<scalar_t>::epsilon();
  scalar_t dt = 0.1;
  scalar_array_t eventTimes{};

  auto time = timeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes);
  ASSERT_EQ(time[0].time, initTime);
  ASSERT_DOUBLE_EQ(time[1].time, initTime + dt);
  ASSERT_EQ(time[2].time, finalTime);  // Absorbs the point at 0.3

  // Events
  ASSERT_EQ(time[0].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[1].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[2].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time.size(), 3);
}

TEST(test_time_discretization, noEvents_minEps) {
  scalar_t initTime = 0.1;
  scalar_t finalTime = 0.3 - std::numeric_limits<scalar_t>::epsilon();
  scalar_t dt = 0.1;
  scalar_array_t eventTimes{};

  auto time = timeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes);
  ASSERT_EQ(time[0].time, initTime);
  ASSERT_DOUBLE_EQ(time[1].time, initTime + dt);
  ASSERT_EQ(time[2].time, finalTime);

  // Events
  ASSERT_EQ(time[0].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[1].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[2].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time.size(), 3);
}

TEST(test_time_discretization, eventAtBeginning) {
  scalar_t initTime = 0.1;
  scalar_t finalTime = 0.3;
  scalar_t dt = 0.1;
  scalar_array_t eventTimes{initTime + std::numeric_limits<scalar_t>::epsilon()};

  auto time = timeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes);
  ASSERT_EQ(time[0].time, eventTimes[0]);
  ASSERT_DOUBLE_EQ(time[1].time, eventTimes[0] + dt);
  ASSERT_EQ(time[2].time, finalTime);  // Absorbs the point at 0.3

  // Events
  ASSERT_EQ(time[0].event, AnnotatedTime::Event::PostEvent);
  ASSERT_EQ(time[1].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[2].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time.size(), 3);
}

TEST(test_time_discretization, withEvents) {
  scalar_t initTime = 3.0;
  scalar_t finalTime = 4.0;
  scalar_t dt = 0.1;
  scalar_array_t eventTimes{3.25, 3.4, 3.8999999999999999999, 4.02, 4.5};

  auto time = timeDiscretizationWithEvents(initTime, finalTime, dt, eventTimes);
  //  timeDiscretization = {3.0, 3.1, 3.2, 3.25, 3.35, 3.4, 3.5, 3.6, 3.7, 3.8, 3.8999999999999999999, 3.98, 4.0}
  ASSERT_EQ(time[0].time, initTime);
  ASSERT_DOUBLE_EQ(time[1].time, initTime + dt);
  ASSERT_DOUBLE_EQ(time[2].time, initTime + 2. * dt);
  ASSERT_EQ(time[3].time, eventTimes[0]);
  ASSERT_EQ(time[4].time, eventTimes[0]);
  ASSERT_DOUBLE_EQ(time[5].time, eventTimes[0] + dt);
  ASSERT_EQ(time[6].time, eventTimes[1]);
  ASSERT_EQ(time[7].time, eventTimes[1]);
  ASSERT_DOUBLE_EQ(time[8].time, eventTimes[1] + dt);
  ASSERT_DOUBLE_EQ(time[9].time, eventTimes[1] + 2. * dt);
  ASSERT_DOUBLE_EQ(time[10].time, eventTimes[1] + 3. * dt);
  ASSERT_DOUBLE_EQ(time[11].time, eventTimes[1] + 4. * dt);
  ASSERT_EQ(time[12].time, eventTimes[2]);
  ASSERT_EQ(time[13].time, eventTimes[2]);
  ASSERT_EQ(time[14].time, finalTime);  // Absorbs 3.8999999999999999999

  // Events
  ASSERT_EQ(time[0].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[1].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[2].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[3].event, AnnotatedTime::Event::PreEvent);
  ASSERT_EQ(time[4].event, AnnotatedTime::Event::PostEvent);
  ASSERT_EQ(time[5].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[6].event, AnnotatedTime::Event::PreEvent);
  ASSERT_EQ(time[7].event, AnnotatedTime::Event::PostEvent);
  ASSERT_EQ(time[8].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[9].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[10].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[11].event, AnnotatedTime::Event::None);
  ASSERT_EQ(time[12].event, AnnotatedTime::Event::PreEvent);
  ASSERT_EQ(time[13].event, AnnotatedTime::Event::PostEvent);
  ASSERT_EQ(time[14].event, AnnotatedTime::Event::None);
}