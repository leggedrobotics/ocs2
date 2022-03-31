/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include "ocs2_mpcnet_core/rollout/MpcnetDataGeneration.h"

#include <random>

#include "ocs2_mpcnet_core/control/MpcnetBehavioralController.h"

namespace ocs2 {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const data_array_t* MpcnetDataGeneration::run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep, size_t dataDecimation,
                                              size_t nSamples, const matrix_t& samplingCovariance,
                                              const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
                                              const TargetTrajectories& targetTrajectories) {
  // clear data array
  dataArray_.clear();

  // set system
  set(alpha, policyFilePath, initialObservation, modeSchedule, targetTrajectories);

  // set up scalar standard normal generator and compute Cholesky decomposition of covariance matrix
  std::random_device randomDevice;
  std::default_random_engine pseudoRandomNumberGenerator(randomDevice());
  std::normal_distribution<scalar_t> standardNormalDistribution(scalar_t(0.0), scalar_t(1.0));
  auto standardNormalNullaryOp = [&](scalar_t) -> scalar_t { return standardNormalDistribution(pseudoRandomNumberGenerator); };
  const matrix_t L = samplingCovariance.llt().matrixL();

  // run data generation
  int iteration = 0;
  try {
    while (systemObservation_.time <= targetTrajectories.timeTrajectory.back()) {
      // step system
      step(timeStep);

      // downsample the data signal by an integer factor
      if (iteration % dataDecimation == 0) {
        // get nominal data point
        const vector_t deviation = vector_t::Zero(primalSolution_.stateTrajectory_.front().size());
        dataArray_.push_back(getDataPoint(*mpcPtr_, *mpcnetDefinitionPtr_, deviation));

        // get samples around nominal data point
        for (int i = 0; i < nSamples; i++) {
          const vector_t deviation = L * vector_t::NullaryExpr(primalSolution_.stateTrajectory_.front().size(), standardNormalNullaryOp);
          dataArray_.push_back(getDataPoint(*mpcPtr_, *mpcnetDefinitionPtr_, deviation));
        }
      }

      // update iteration
      ++iteration;
    }
  } catch (const std::exception& e) {
    // print error for exceptions
    std::cerr << "[MpcnetDataGeneration::run] a standard exception was caught, with message: " << e.what() << "\n";
    // this data generation run failed, clear data
    dataArray_.clear();
  }

  // return pointer to the data array
  return &dataArray_;
}

}  // namespace mpcnet
}  // namespace ocs2
