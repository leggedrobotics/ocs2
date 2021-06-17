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

#include <algorithm>

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_mpc/MPC_BASE.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_BASE::MPC_BASE(mpc::Settings mpcSettings) : mpcSettings_(std::move(mpcSettings)), nextTimeHorizon_(mpcSettings_.timeHorizon_) {
  if (mpcSettings_.numPartitions_ == 0) {
    throw std::runtime_error("There should be at least one time partition.");
  }

  // Initialize partition times
  partitionTimes_ = initializePartitionTimes(mpcSettings_.timeHorizon_, mpcSettings_.numPartitions_);
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
  // Shift partition times forward, discard the first time block.
  auto lastPartition = std::copy(partitionTimes_.begin() + mpcSettings_.numPartitions_, partitionTimes_.end(), partitionTimes_.begin());

  // Update last partition time block, timeHorizon_ can have changed since the last rewind.
  mpcSettings_.timeHorizon_ = nextTimeHorizon_;
  auto t = partitionTimes_.back();
  std::generate(lastPartition, partitionTimes_.end(), [&]() {
    t += mpcSettings_.timeHorizon_ / mpcSettings_.numPartitions_;
    return t;
  });

  // Solver internal variables
  getSolverPtr()->rewindOptimizer(mpcSettings_.numPartitions_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_BASE::run(scalar_t currentTime, const vector_t& currentState) {
  // check if the current time exceeds the solver final limit
  if (!initRun_ && currentTime >= getSolverPtr()->getFinalTime()) {
    std::cerr << "WARNING: The MPC time-horizon is smaller than the MPC starting time.\n";
    std::cerr << "currentTime: " << currentTime << "\t Controller finalTime: " << getSolverPtr()->getFinalTime() << '\n';
    return false;
  }

  // Check if a goal has been set
  if (initRun_ && getSolverPtr()->getReferenceManager().getTargetTrajectories().empty()) {
    std::cerr << "### WARNING: The initial TargetTrajectories are not set. This may cause undefined behavior. "
                 "Use the MPC_BASE::getSolverPtr()->getReferenceManager().setTargetTrajectories() method "
                 "to provide target trajectories.\n";
  }

  // adjusting the partitioning times based on the initial time
  if (initRun_) {
    const scalar_t deltaTime = currentTime - partitionTimes_[mpcSettings_.numPartitions_];
    for (auto& t : partitionTimes_) {
      t += deltaTime;
    }
  }

  scalar_t finalTime = currentTime + mpcSettings_.timeHorizon_;

  // display
  if (mpcSettings_.debugPrint_) {
    std::cerr << "\n#####################################################";
    std::cerr << "\n#####################################################";
    std::cerr << "\n#####################################################";
    std::cerr << "\n### MPC is called at time:  " << currentTime << " [s].";
    std::cerr << "\n### MPC final Time:         " << finalTime << " [s].";
    std::cerr << "\n### MPC time horizon:       " << mpcSettings_.timeHorizon_ << " [s].\n";
    mpcTimer_.startTimer();
  }

  // rewind the optimizer
  if (finalTime > partitionTimes_.back()) {
    if (mpcSettings_.debugPrint_) {
      std::cerr << "### MPC is rewinded at time " << currentTime << " [s].\n";
    }
    rewind();
  }

  // Adjust the initial and final time based on the partition times (must be after rewinding!)
  adjustTimeHorizon(partitionTimes_, currentTime, finalTime);

  // calculate the MPC policy
  calculateController(currentTime, currentState, finalTime);

  // display
  if (mpcSettings_.debugPrint_) {
    mpcTimer_.endTimer();
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms].";
    std::cerr << "\n###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

  // set initRun flag to false
  initRun_ = false;
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_array_t MPC_BASE::initializePartitionTimes(scalar_t timeHorizon, size_t numPartitions) {
  // Create partition times over 3 timeHorizon
  // partitionTimes = [ -T, ... , 0.0, ... T, ... 2T ]

  scalar_array_t partitionTimes;
  partitionTimes.resize(3 * numPartitions + 1);
  partitionTimes[0] = -timeHorizon;
  for (size_t i = 0; i < 3 * numPartitions; i++) {
    partitionTimes[i + 1] = partitionTimes[i] + timeHorizon / numPartitions;
  }

  return partitionTimes;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::adjustTimeHorizon(const scalar_array_t& partitionTimes, scalar_t& initTime, scalar_t& finalTime) {
  // TODO(mspieler): Workaround for when initTime and finalTime are close to a partition boundary.
  //                 Times are rounded towards a smaller time horizon to avoid very short partitions.
  const scalar_t partitionTimeTolerance = 4e-3;       //! @badcode magic epsilon
  const scalar_t deltaTimePastFirstPartition = 1e-5;  //! @badcode magic epsilon

  // current active subsystem
  const auto initActivePartitionIndex = lookup::findBoundedActiveIntervalInTimeArray(partitionTimes, initTime);
  const auto initialActivePartitionTime = partitionTimes[initActivePartitionIndex + 1];

  // If the initial time is right before the start of the first partition, set it past the start of that partition
  if (initTime > initialActivePartitionTime - partitionTimeTolerance) {
    initTime = initialActivePartitionTime + deltaTimePastFirstPartition;
  }

  // final active subsystem
  const auto finalActivePartitionIndex = lookup::findBoundedActiveIntervalInTimeArray(partitionTimes, finalTime);
  const auto finalActivePartitionTime = partitionTimes[initActivePartitionIndex + 1];

  // If the final time is right after the final partition, shrink the horizon the end at that partition.
  if (finalTime < finalActivePartitionTime + partitionTimeTolerance) {
    finalTime = finalActivePartitionTime;
  }
}

}  // namespace ocs2
