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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

using namespace ocs2;

TEST(time_rollout_test, time_rollout_test) {
  const size_t nx = 2;
  const size_t nu = 1;
  scalar_t initTime = 0.0;
  scalar_t finalTime = 10.0;

  Eigen::Matrix2d A(nx, nx);
  A << -2, -1, 1, 0;
  Eigen::Vector2d B(nx, nu);
  B << 1, 0;

  LinearSystemDynamics systemDynamics(A, B);

  // controller
  scalar_array_t cntTimeStamp{initTime, finalTime};
  vector_array_t uff(2, vector_t::Ones(nu));
  matrix_array_t k(2, matrix_t::Zero(nu, nx));
  auto controller = std::unique_ptr<LinearController>(new LinearController(cntTimeStamp, uff, k));

  vector_t initState = vector_t::Zero(nx);

  // partitioning times
  std::vector<scalar_t> partitioningTimes{0.0, 4.0, 5.0, 7.0};

  // event times
  std::vector<scalar_t> eventTimes = std::vector<scalar_t>{3.0, 4.0, 4.0};

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // Rollout Settings
  rollout::Settings rolloutSettings;
  rolloutSettings.absTolODE = 1e-7;
  rolloutSettings.relTolODE = 1e-5;
  rolloutSettings.timeStep = 1e-3;
  rolloutSettings.maxNumStepsPerSecond = 10000;

  // rollout class
  std::unique_ptr<RolloutBase> rolloutBasePtr(new TimeTriggeredRollout(systemDynamics, rolloutSettings));

  scalar_array_t timeTrajectory;
  size_array_t eventsPastTheEndIndeces;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;

  size_t partitionIndex = 0;
  rolloutBasePtr->run(initTime, initState, finalTime, controller.get(), eventTimes, timeTrajectory, eventsPastTheEndIndeces,
                      stateTrajectory, inputTrajectory);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // check sizes
  const auto totalSize = timeTrajectory.size();
  ASSERT_EQ(totalSize, stateTrajectory.size());
  ASSERT_EQ(totalSize, inputTrajectory.size());
}
