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
// C++
#include <cstdlib>
#include <iostream>
#include <string>

#include <ros/package.h>

// OCS2
#include <ocs2_core/Dimensions.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

// Ballbot
#include "ocs2_ballbot_example/cost/BallbotCost.h"
#include "ocs2_ballbot_example/definitions.h"
#include "ocs2_ballbot_example/dynamics/BallbotSystemDynamics.h"
#include "ocs2_ballbot_example/solvers/BallbotSLQ.h"

using namespace ocs2;
using namespace ballbot;

int main(int argc, char** argv) {
  using dim_t = Dimensions<STATE_DIM_, INPUT_DIM_>;
  using ballbotConstraint_t = ConstraintBase<STATE_DIM_, INPUT_DIM_>;
  using ballbotOperatingPoint_t = SystemOperatingPoint<STATE_DIM_, INPUT_DIM_>;

  /*
   * Setting paths
   */
  // path to config file
  std::string taskFile = ros::package::getPath("ocs2_ballbot_example") + "/config/mpc/task.info";
  std::cerr << "Loading task file: " << taskFile << std::endl;
  // path to save auto-generated libraries
  std::string libraryFolder = ros::package::getPath("ocs2_ballbot_example") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder << std::endl;

  /*
   * SLQ settings
   */
  SLQ_Settings slqSettings;
  slqSettings.loadSettings(taskFile);
  slqSettings.ddpSettings_.displayInfo_ = true;  // display iteration information

  /*
   * Rollout settings
   */
  Rollout_Settings rolloutSettings;
  rolloutSettings.loadSettings(taskFile, "slq.rollout");

  /*
   * Rollout
   */
  std::unique_ptr<BallbotSystemDynamics> ballbotSystemDynamicsPtr(new BallbotSystemDynamics());
  ballbotSystemDynamicsPtr->initialize("ballbot_dynamics", libraryFolder, true, true);
  std::unique_ptr<TimeTriggeredRollout<STATE_DIM_, INPUT_DIM_>> ballbotRolloutPtr(
      new TimeTriggeredRollout<STATE_DIM_, INPUT_DIM_>(*ballbotSystemDynamicsPtr, rolloutSettings));

  /*
   * Cost function
   */
  dim_t::state_matrix_t Q;
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  dim_t::input_matrix_t R;
  loadData::loadEigenMatrix(taskFile, "R", R);
  dim_t::state_matrix_t QFinal;
  loadData::loadEigenMatrix(taskFile, "Q_final", QFinal);
  dim_t::state_vector_t xFinal;
  loadData::loadEigenMatrix(taskFile, "x_final", xFinal);
  dim_t::state_vector_t xInit;
  loadData::loadEigenMatrix(taskFile, "initialState", xInit);
  dim_t::state_vector_t xNominal = xInit;
  dim_t::input_vector_t uNominal = dim_t::input_vector_t::Zero();

  std::unique_ptr<BallbotCost> ballbotCostPtr(new BallbotCost(Q, R, xNominal, uNominal, QFinal, xFinal));

  /*
   * Constraints
   */
  std::unique_ptr<ballbotConstraint_t> ballbotConstraintPtr(new ballbotConstraint_t);

  /*
   * Initialization
   */
  std::unique_ptr<ballbotOperatingPoint_t> ballbotOperatingPointPtr(new ballbotOperatingPoint_t(xInit, dim_t::input_vector_t::Zero()));

  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  dim_t::scalar_t timeHorizon;
  loadData::loadCppDataType(taskFile, "mpcTimeHorizon.timehorizon", timeHorizon);
  size_t numPartitions;
  loadData::loadCppDataType(taskFile, "mpcTimeHorizon.numPartitions", numPartitions);
  dim_t::scalar_array_t partitioningTimes(numPartitions + 1);
  partitioningTimes[0] = 0.0;
  for (size_t i = 0; i < numPartitions; i++) {
    partitioningTimes[i + 1] = partitioningTimes[i] + timeHorizon / numPartitions;
  }
  partitioningTimes[numPartitions] = timeHorizon;

  /*
   * define solver and run
   */
  slqSettings.ddpSettings_.nThreads_ = 1;
  SLQ<STATE_DIM_, INPUT_DIM_> slqST(ballbotRolloutPtr.get(), ballbotSystemDynamicsPtr.get(), ballbotConstraintPtr.get(),
                                    ballbotCostPtr.get(), ballbotOperatingPointPtr.get(), slqSettings);
  slqST.run(0.0, xInit, timeHorizon, partitioningTimes);

  /*
   * Perforce index
   */
  auto performanceIndex = slqST.getPerformanceIndeces();

  return 0;
}
