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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

#include "ocs2_oc/rollout/TimeTriggeredRollout.h"
#include "ocs2_oc/test/EXP1.h"

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 1 };

TEST(time_rollout_test, time_rollout_test) {
  double initTime = 0.0;
  double finalTime = 10.0;

  Eigen::Matrix2d A;
  A << -2, -1, 1, 0;
  Eigen::Vector2d B;
  B << 1, 0;

  typedef LinearSystemDynamics<STATE_DIM, INPUT_DIM> SecondOrderSystem;
  SecondOrderSystem systemDynamics(A, B);

  // controller
  SecondOrderSystem::scalar_array_t cntTimeStamp{initTime, finalTime};
  SecondOrderSystem::input_vector_array_t uff(2, SecondOrderSystem::input_vector_t::Ones());
  SecondOrderSystem::input_state_matrix_array_t k(2, SecondOrderSystem::input_state_matrix_t::Zero());
  using controller_t = ocs2::LinearController<STATE_DIM, INPUT_DIM>;
  auto controller = std::unique_ptr<controller_t>(new controller_t(cntTimeStamp, uff, k));

  SecondOrderSystem::state_vector_t initState;
  initState.setZero();

  // partitioning times
  std::vector<double> partitioningTimes{0.0, 4.0, 5.0, 7.0};

  // event times
  std::vector<double> eventTimes = std::vector<double>{3.0, 4.0, 4.0};

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // Rollout Settings
  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-7;
  rolloutSettings.relTolODE_ = 1e-5;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  // rollout class
  using rollout_base_t = RolloutBase<STATE_DIM, INPUT_DIM>;
  std::unique_ptr<rollout_base_t> rolloutBasePtr(new TimeTriggeredRollout<STATE_DIM, INPUT_DIM>(systemDynamics, rolloutSettings));

  rollout_base_t::scalar_array_t timeTrajectory;
  rollout_base_t::size_array_t eventsPastTheEndIndeces;
  rollout_base_t::state_vector_array_t stateTrajectory;
  rollout_base_t::input_vector_array_t inputTrajectory;
  ModelDataBase::array_t modelDataTrajectory;

  size_t partitionIndex = 0;
  rolloutBasePtr->run(initTime, initState, finalTime, controller.get(), eventTimes, timeTrajectory, eventsPastTheEndIndeces,
                      stateTrajectory, inputTrajectory, &modelDataTrajectory);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // check sizes
  const auto totalSize = timeTrajectory.size();
  ASSERT_EQ(totalSize, stateTrajectory.size());
  ASSERT_EQ(totalSize, inputTrajectory.size());
  ASSERT_EQ(totalSize, modelDataTrajectory.size());

  // check model data trajectory
  for (const auto& modelData : modelDataTrajectory) {
    ASSERT_EQ(modelData.stateDim_, stateTrajectory.front().rows());
    ASSERT_EQ(modelData.inputDim_, inputTrajectory.front().rows());
    ASSERT_EQ(modelData.flowMap_.rows(), stateTrajectory.front().rows());
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
