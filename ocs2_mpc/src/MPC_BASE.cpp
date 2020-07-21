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

#include <ocs2_core/misc/Lookup.h>

#include <ocs2_mpc/MPC_BASE.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_BASE::MPC_BASE(const scalar_array_t& partitioningTimes, MPC_Settings mpcSettings)

    : mpcSettings_(std::move(mpcSettings)), initnumPartitions_(partitioningTimes.size() - 1), initPartitioningTimes_(partitioningTimes) {
  if (partitioningTimes.size() < 2) {
    throw std::runtime_error("There should be at least one time partition.");
  }

  // Initialize partition times
  partitioningTimes_ = initializePartitionTimes(initPartitioningTimes_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::reset() {
  initRun_ = true;
  mpcTimer_.reset();
  getSolverPtr()->reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::rewind() {
  // Shift all partition times by 2 time horizons
  const scalar_t timeHorizon = getTimeHorizon();
  for (auto& t : partitioningTimes_) {
    t += 2 * timeHorizon;
  }

  // Solver internal variables
  getSolverPtr()->rewindOptimizer(initnumPartitions_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_BASE::run(scalar_t currentTime, const vector_t& currentState) {
  // check if the current time exceeds the solver final limit
  if (!initRun_ && currentTime >= getSolverPtr()->getFinalTime()) {
    std::cerr << "WARNING: The MPC time-horizon is smaller than the MPC starting time." << std::endl;
    std::cerr << "currentTime: " << currentTime << "\t Controller finalTime: " << getSolverPtr()->getFinalTime() << std::endl;
    return false;
  }

  // Check if a goal has been set
  if (initRun_ && getSolverPtr()->getCostDesiredTrajectories().empty()) {
    std::cerr << "### WARNING: The initial desired trajectories are not set. "
                 "This may cause undefined behavior. Use the MPC_BASE::getSolverPtr()->setCostDesiredTrajectories() "
                 "method to provide appropriate goal trajectories."
              << std::endl;
  }

  // adjusting the partitioning times based on the initial time
  if (initRun_) {
    const scalar_t deltaTime = currentTime - partitioningTimes_[initnumPartitions_];
    for (auto& t : partitioningTimes_) {
      t += deltaTime;
    }
  }

  // rewind the optimizer
  scalar_t finalTime = currentTime + getTimeHorizon();
  if (finalTime > partitioningTimes_.back()) {
    if (mpcSettings_.debugPrint_) {
      std::cerr << "### MPC is rewinded at time " << currentTime << " [s]." << std::endl;
    }
    rewind();
  }

  // Adjust the initial and final time based on the partition times (must be after rewinding!)
  scalar_t initTime = currentTime;
  adjustmentTimeHorizon(partitioningTimes_, initTime, finalTime);

  // display
  if (mpcSettings_.debugPrint_) {
    std::cerr << "#####################################################\n";
    std::cerr << "#####################################################\n";
    std::cerr << "#####################################################\n";
    std::cerr << "### MPC is called at time:  " << currentTime << " [s].\n";
    std::cerr << "### MPC final Time:         " << finalTime << " [s].\n";
    std::cerr << "### MPC time horizon:       " << getTimeHorizon() << " [s]." << std::endl;
    mpcTimer_.startTimer();
  }

  // calculate the MPC policy
  calculateController(initTime, currentState, finalTime);

  // display
  if (mpcSettings_.debugPrint_) {
    mpcTimer_.endTimer();
    std::cerr << "### MPC runtime \n";
    std::cerr << "###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].\n";
    std::cerr << "###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].\n";
    std::cerr << "###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

  // set initRun flag to false
  initRun_ = false;
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto MPC_BASE::initializePartitionTimes(const scalar_array_t& initPartitionTimes) const -> scalar_array_t {
  if (initPartitionTimes.size() < 2) {
    throw std::runtime_error("[MPC_BASE] There should be at least one time partition.");
  }
  scalar_array_t partitionTimes;

  // Add partition times before and after the current partition times
  // Create partition times with [{initPartitionTimes - T}, {initPartitionTimes}, {initPartitionTimes + T}]
  const scalar_t timeHorizon = initPartitionTimes.back() - initPartitionTimes.front();
  for (int j = -1; j <= 1; j++) {
    for (int i = 0; (i + 1) < initPartitionTimes.size(); i++) {
      partitionTimes.push_back(j * timeHorizon + initPartitionTimes[i]);
    }
  }
  partitionTimes.push_back(initPartitionTimes.back() + timeHorizon);
  return partitionTimes;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::adjustmentTimeHorizon(const scalar_array_t& partitioningTimes, scalar_t& initTime, scalar_t& finalTime) const {
  const scalar_t partitionTimeTolerance = 4e-3;       //! @badcode magic epsilon
  const scalar_t deltaTimePastFirstPartition = 1e-5;  //! @badcode magic epsilon

  // current active subsystem
  const auto initActivePartitionIndex = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, initTime);
  const auto initialActivePartitionTime = partitioningTimes[initActivePartitionIndex + 1];

  // If the initial time is right before the start of the first partition, set it past the start of that partition
  if (initTime > initialActivePartitionTime - partitionTimeTolerance) {
    initTime = initialActivePartitionTime + deltaTimePastFirstPartition;
  }

  // final active subsystem
  const auto finalActivePartitionIndex = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, finalTime);
  const auto finalActivePartitionTime = partitioningTimes[initActivePartitionIndex + 1];

  // If the final time is right after the final partition, shrink the horizon the end at that partition.
  if (finalTime < finalActivePartitionTime + partitionTimeTolerance) {
    finalTime = finalActivePartitionTime;
  }
}

}  // namespace ocs2
