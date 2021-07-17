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
#include <ocs2_core/Types.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

// Ballbot
#include "ocs2_ballbot/definitions.h"
#include "ocs2_ballbot/dynamics/BallbotSystemDynamics.h"

using namespace ocs2;
using namespace ballbot;

int main(int argc, char** argv) {
  /*
   * Setting paths
   */
  // path to config file
  std::string taskFile = ros::package::getPath("ocs2_ballbot") + "/config/mpc/task.info";
  std::cerr << "Loading task file: " << taskFile << std::endl;
  // path to save auto-generated libraries
  std::string libraryFolder = ros::package::getPath("ocs2_ballbot") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder << std::endl;

  /* The optimal control problem formulation*/
  OptimalControlProblem problem;

  /*
   * DDP settings
   */
  auto ddpSettings = ddp::loadSettings(taskFile, "ddp");
  ddpSettings.displayInfo_ = true;  // display iteration information

  /*
   * Rollout settings
   */
  auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");

  /*
   * Rollout
   */
  problem.dynamicsPtr.reset(new BallbotSystemDynamics(libraryFolder, true));
  std::unique_ptr<TimeTriggeredRollout> ballbotRolloutPtr(new TimeTriggeredRollout(*problem.dynamicsPtr, rolloutSettings));

  /*
   * Cost function
   */
  matrix_t Q(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R(INPUT_DIM, INPUT_DIM);
  loadData::loadEigenMatrix(taskFile, "R", R);
  matrix_t QFinal(STATE_DIM, STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "Q_final", QFinal);
  vector_t xInit(STATE_DIM);
  loadData::loadEigenMatrix(taskFile, "initialState", xInit);

  std::unique_ptr<QuadraticStateInputCost> L(new QuadraticStateInputCost(Q, R));
  std::unique_ptr<QuadraticStateCost> Phi(new QuadraticStateCost(QFinal));
  problem.costPtr->add("cost", std::move(L));
  problem.finalCostPtr->add("finalCost", std::move(Phi));

  /*
   * Initialization
   */
  std::unique_ptr<Initializer> ballbotInitializerPtr(new DefaultInitializer(INPUT_DIM));

  /*
   * Time partitioning which defines the time horizon and the number of data partitioning
   */
  scalar_t timeHorizon;
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", timeHorizon);
  size_t numPartitions;
  loadData::loadCppDataType(taskFile, "mpc.numPartitions", numPartitions);
  scalar_array_t partitioningTimes(numPartitions + 1);
  partitioningTimes[0] = 0.0;
  for (size_t i = 0; i < numPartitions; i++) {
    partitioningTimes[i + 1] = partitioningTimes[i] + timeHorizon / numPartitions;
  }
  partitioningTimes[numPartitions] = timeHorizon;

  /*
   * define solver and run
   */
  ddpSettings.nThreads_ = 1;
  SLQ slq(ddpSettings, *ballbotRolloutPtr, problem, *ballbotInitializerPtr);
  slq.getReferenceManager().setTargetTrajectories(TargetTrajectories({0.0}, {xInit}, {vector_t::Zero(INPUT_DIM)}));
  slq.run(0.0, xInit, timeHorizon, partitioningTimes);

  /*
   * Perforce index
   */
  auto performanceIndex = slq.getPerformanceIndeces();

  return 0;
}
