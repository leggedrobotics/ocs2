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
MPC_BASE::MPC_BASE()

    : stateDim_(0),
      inputDim_(0),
      initRun_(true),
      initnumPartitions_(0),
      initPartitioningTimes_(0),
      numPartitions_(0),
      partitioningTimes_(0),
      initActivePartitionIndex_(0),
      finalActivePartitionIndex_(0),
      lastControlDesignTime_(0.0),
      solverPtr_(nullptr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_BASE::MPC_BASE(size_t stateDim, size_t inputDim, const scalar_array_t& partitioningTimes, MPC_Settings mpcSettings)

    : mpcSettings_(std::move(mpcSettings)),
      stateDim_(stateDim),
      inputDim_(inputDim),
      initRun_(true),
      initnumPartitions_(partitioningTimes.size() - 1),
      initPartitioningTimes_(partitioningTimes),
      numPartitions_(0),
      partitioningTimes_(0),
      initActivePartitionIndex_(0),
      finalActivePartitionIndex_(0),
      lastControlDesignTime_(partitioningTimes.front()),
      solverPtr_(nullptr) {
  if (partitioningTimes.size() < 2) {
    throw std::runtime_error("There should be at least one time partition.");
  }

  if (mpcSettings_.recedingHorizon_) {
    numPartitions_ = 3 * initnumPartitions_;
    partitioningTimes_.clear();

    const scalar_t timeHorizon = initPartitioningTimes_.back() - initPartitioningTimes_.front();

    for (int j = 0; j < 3; j++) {
      for (int i = 0; i < initnumPartitions_; i++) {
        partitioningTimes_.push_back((j - 1) * timeHorizon + initPartitioningTimes_[i]);
      }  // end of i loop
    }    // end of j loop
    partitioningTimes_.push_back(initPartitioningTimes_[initnumPartitions_] + timeHorizon);

  } else {
    numPartitions_ = initnumPartitions_;
    partitioningTimes_ = initPartitioningTimes_;
  }

  // correcting solutionTimeWindow
  if (mpcSettings_.recedingHorizon_) {
    mpcSettings_.solutionTimeWindow_ = -1;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::reset() {
  initRun_ = true;

  mpcTimer_.reset();

  solverPtr_->reset();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::setBaseSolverPtr(Solver_BASE* solverPtr) {
  solverPtr_ = solverPtr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::rewind() {
  for (size_t i = 0; i < 2 * initnumPartitions_; i++) {
    partitioningTimes_[i] = partitioningTimes_[i + initnumPartitions_];
  }

  const scalar_t timeHorizon = initPartitioningTimes_.back() - initPartitioningTimes_.front();
  for (size_t i = 0; i < initnumPartitions_; i++) {
    partitioningTimes_[i + 2 * initnumPartitions_] = 2.0 * timeHorizon + partitioningTimes_[i];
  }
  partitioningTimes_[3 * initnumPartitions_] = 3.0 * timeHorizon + partitioningTimes_[0];

  // Solver internal variables
  solverPtr_->rewindOptimizer(initnumPartitions_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::adjustmentTimeHorizon(const scalar_array_t& partitioningTimes, scalar_t& initTime, scalar_t& finalTime,
                                     size_t& initActivePartitionIndex, size_t& finalActivePartitionIndex) const {
  // current active subsystem
  initActivePartitionIndex = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, initTime);

  if (initTime > partitioningTimes[initActivePartitionIndex + 1] - 4e-3) {  //! @badcode magic epsilon
    initTime = partitioningTimes[initActivePartitionIndex + 1] + 1e-5;      //! @badcode magic epsilon
    initActivePartitionIndex++;
  }

  // final active subsystem
  finalActivePartitionIndex = lookup::findBoundedActiveIntervalInTimeArray(partitioningTimes, finalTime);

  // if it is at the very beginning of the partition (4e-3) reduce the final time to
  // last partition final time otherwise set to the final time of the current partition
  // (if blockwiseMovingHorizon is active)
  if (finalTime < partitioningTimes[finalActivePartitionIndex] + 4e-3) {  //! @badcode magic epsilon
    finalTime = partitioningTimes[finalActivePartitionIndex];
    finalActivePartitionIndex--;
  } else {
    if (mpcSettings_.blockwiseMovingHorizon_) {
      finalTime = partitioningTimes[finalActivePartitionIndex + 1];
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_BASE::run(const scalar_t& currentTime, const vector_t& currentState) {
  // check if the current time exceeds the solver final limit
  if (!initRun_ && currentTime >= getFinalTime() && mpcSettings_.recedingHorizon_) {
    std::cerr << std::endl << "#####################################################";
    std::cerr << std::endl << "#####################################################";
    std::cerr << std::endl << "#####################################################" << std::endl;
    std::cerr << "### MPC is called at time:  " << currentTime << " [s]." << std::endl;
    std::cerr << "WARNING: The MPC time-horizon is smaller than the MPC starting time." << std::endl;
    std::cerr << "currentTime: " << currentTime << "\t Controller finalTime: " << getFinalTime() << std::endl;

    return false;
  }

  // adjusting the partitioning times based on the initial time
  if (initRun_) {
    const scalar_t detaTime = currentTime - partitioningTimes_[initnumPartitions_];
    for (auto& time : partitioningTimes_) {
      time += detaTime;
    }
  }

  // display
  if (mpcSettings_.debugPrint_) {
    std::cerr << std::endl << "#####################################################";
    std::cerr << std::endl << "#####################################################";
    std::cerr << std::endl << "#####################################################" << std::endl;
    std::cerr << "### MPC is called at time:  " << currentTime << " [s]." << std::endl;
    // CPU time at the beginning MPC
    mpcTimer_.startTimer();
  }

  /******************************************************************************************
   * Determine MPC time horizon
   ******************************************************************************************/
  scalar_t initTime = currentTime;
  scalar_t finalTime;
  if (mpcSettings_.recedingHorizon_) {
    finalTime = currentTime + getTimeHorizon();
  } else {
    auto N = static_cast<size_t>(currentTime / getFinalTime());
    finalTime = (N + 1) * getFinalTime();
  }

  if (mpcSettings_.debugPrint_) {
    std::cerr << "### MPC final Time:         " << finalTime << " [s]." << std::endl;
    std::cerr << "### MPC time horizon:       " << finalTime - currentTime << " [s]." << std::endl;
  }

  /******************************************************************************************
   * rewind the optimizer
   ******************************************************************************************/
  if (finalTime > partitioningTimes_.back() /*&& BASE::mpcSettings_.recedingHorizon_==true*/) {
    if (mpcSettings_.debugPrint_) {
      std::cerr << "### MPC is rewinded at time " << currentTime << " [s]." << std::endl;
    }
    rewind();
  }

  /******************************************************************************************
   * time horizon adjustment
   ******************************************************************************************/
  adjustmentTimeHorizon(partitioningTimes_, initTime, finalTime, initActivePartitionIndex_, finalActivePartitionIndex_);

  lastControlDesignTime_ = initTime;

  // display
  if (mpcSettings_.debugPrint_) {
    std::cerr << "### MPC number of subsystems: " << finalActivePartitionIndex_ - initActivePartitionIndex_ + 1 << "." << std::endl;
    std::cerr << "### MPC time horizon is adjusted." << std::endl;
    std::cerr << "\t### MPC final Time:   " << finalTime << " [s]." << std::endl;
    std::cerr << "\t### MPC time horizon: " << finalTime - initTime << " [s]." << std::endl;
  }

  /******************************************************************************************
   * cost goal check
   ******************************************************************************************/
  if (initRun_ && solverPtr_->getCostDesiredTrajectories().empty()) {
    std::cerr << "### WARNING: The initial desired trajectories are not set. "
                 "This may cause undefined behavior. Use the MPC_SLQ::setCostDesiredTrajectories() "
                 "method to provide appropriate goal trajectories."
              << std::endl;
  }

  /******************************************************************************************
   * Calculate controller
   ******************************************************************************************/
  // calculate the MPC policy
  calculateController(initTime, currentState, finalTime);

  // set initRun flag to false
  initRun_ = false;

  // display
  if (mpcSettings_.debugPrint_) {
    // updating runtime of the MPC for adaptive frequency
    mpcTimer_.endTimer();
    std::cerr << "### MPC runtime " << std::endl;
    std::cerr << "###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "###   Latest  : " << mpcTimer_.getLastIntervalInMilliseconds() << "[ms]." << std::endl;
  }

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MPC_BASE::getStartTime() const {
  return lastControlDesignTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MPC_BASE::getFinalTime() const {
  if (mpcSettings_.recedingHorizon_) {
    return lastControlDesignTime_ + getTimeHorizon();
  } else {
    return initPartitioningTimes_.back();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MPC_BASE::getTimeHorizon() const {
  if (mpcSettings_.recedingHorizon_) {
    return initPartitioningTimes_.back() - initPartitioningTimes_.front();
  } else {
    return initPartitioningTimes_.back() - lastControlDesignTime_;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_BASE::getPartitioningTimes(scalar_array_t& partitioningTimes) const {
  partitioningTimes.resize(finalActivePartitionIndex_ + 2);
  for (size_t i = 0; i <= finalActivePartitionIndex_ + 1; i++) {
    partitioningTimes[i] = partitioningTimes_[i];
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const MPC_Settings& MPC_BASE::settings() const {
  return mpcSettings_;
}

}  // namespace ocs2
