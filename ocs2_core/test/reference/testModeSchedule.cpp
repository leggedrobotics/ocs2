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
#include <type_traits>
#include <typeinfo>

#include <gtest/gtest.h>

#include <ocs2_core/NumericTraits.h>
#include <ocs2_core/reference/ModeSchedule.h>

using namespace ocs2;

TEST(testModeSchedule, getNumberOfPrecedingEvents) {
  constexpr auto eps = numeric_traits::limitEpsilon<scalar_t>();
  const scalar_array_t eventTimes{1.0, 2.0, 3.0, 4.0, 5.0};
  scalar_array_t timeTrajectory{1.5, 2.0, (2.0 + eps), 2.5, 3.0, (3.0 + eps), 3.5};
  size_array_t postEventIndices{2, 5};
  size_t output;
  scalar_t eventTime;

  // event before
  eventTime = 1.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 0);

  // event after
  eventTime = 5.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 2);

  // first event
  eventTime = 2.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 0);

  // second event
  eventTime = 3.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 1);

  // not an event and with the span of timeTrajectory
  eventTime = 2.3;
  EXPECT_THROW(getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime), std::runtime_error);

  // event at final time not included (like the situation in solvers)
  timeTrajectory = scalar_array_t{1.5, 2.0, (2.0 + eps), 2.5, 3.0};
  postEventIndices = size_array_t{2};
  eventTime = 3.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 1);

  // event at initial time not included (like the situation in solvers)
  timeTrajectory = scalar_array_t{2.0, 2.5, 3.0, (3.0 + eps), 3.5};
  postEventIndices = size_array_t{3};
  eventTime = 2.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 0);
  eventTime = 3.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 0);
  eventTime = 4.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 1);

  // event at initial time but included
  timeTrajectory = scalar_array_t{2.0, (2.0 + eps), 2.5, 3.0, (3.0 + eps), 3.5};
  postEventIndices = size_array_t{1, 4};
  eventTime = 3.0;
  output = getNumberOfPrecedingEvents(timeTrajectory, postEventIndices, eventTime);
  EXPECT_EQ(output, 1);
}

TEST(testModeSchedule, findIntersectionToExtendableInterval) {
  const scalar_array_t eventTimes{1.0, 2.0, 3.0, 4.0};
  scalar_array_t timeTrajectory;
  std::pair<scalar_t, scalar_t> timePeriod;
  std::pair<scalar_t, scalar_t> output;

  // empty timeTrajectory
  timeTrajectory.clear();
  timePeriod = std::make_pair(1.5, 3.0);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // empty interval
  timeTrajectory.clear();
  timePeriod = std::make_pair(1.5, 1.0);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // the same as timeTrajectory
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(timeTrajectory.front(), timeTrajectory.back());
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_EQ(output.first, timeTrajectory.front());
  EXPECT_EQ(output.second, timeTrajectory.back());

  // with timeTrajectory
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(1.6, 2.0);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_EQ(output.first, 1.6);
  EXPECT_EQ(output.second, 2.0);

  // no intersection to timeTrajectory but its extendable interval (left hand side case)
  timeTrajectory = scalar_array_t{2.5, 3.3};
  timePeriod = std::make_pair(1.8, 2.1);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_EQ(output.first, 2.0);
  EXPECT_EQ(output.second, 2.1);

  // no intersection to timeTrajectory but its extendable interval (right hand side case)
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(2.8, 3.8);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_EQ(output.first, 2.8);
  EXPECT_EQ(output.second, 3.0);

  // no intersection to timeTrajectory and its extendable interval (left hand side case)
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(0.0, 1.0);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // no intersection to timeTrajectory and its extendable interval (right hand side case)
  timeTrajectory = scalar_array_t{1.5, 2.3};
  timePeriod = std::make_pair(3.0, 3.8);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // corner case: no intersection time trajectory end at event time (left hand side case)
  timeTrajectory = scalar_array_t{2.0, 4.0};
  timePeriod = std::make_pair(0.0, 2.0);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";

  // corner case: no intersection time trajectory end at event time (right hand side case)
  timeTrajectory = scalar_array_t{1.0, 3.0};
  timePeriod = std::make_pair(3.0, 4.0);
  output = findIntersectionToExtendableInterval(timeTrajectory, eventTimes, timePeriod);
  EXPECT_TRUE(output.first > output.second) << "The interval should be empty!";
}
