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

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/reference/ModeSchedule.h>

namespace ocs2 {

/**
 * This class contains the primal problem's solution.
 */
struct PrimalSolution {
  /** Constructor */
  PrimalSolution() = default;

  /** Destructor */
  ~PrimalSolution() = default;

  /** Copy constructor */
  PrimalSolution(const PrimalSolution& other)
      : timeTrajectory_(other.timeTrajectory_),
        stateTrajectory_(other.stateTrajectory_),
        inputTrajectory_(other.inputTrajectory_),
        postEventIndices_(other.postEventIndices_),
        modeSchedule_(other.modeSchedule_),
        controllerPtr_(other.controllerPtr_ ? other.controllerPtr_->clone() : nullptr) {}

  /** Copy Assignment */
  PrimalSolution& operator=(const PrimalSolution& other) {
    timeTrajectory_ = other.timeTrajectory_;
    stateTrajectory_ = other.stateTrajectory_;
    inputTrajectory_ = other.inputTrajectory_;
    postEventIndices_ = other.postEventIndices_;
    modeSchedule_ = other.modeSchedule_;
    if (other.controllerPtr_) {
      controllerPtr_.reset(other.controllerPtr_->clone());
    } else {
      controllerPtr_.reset();
    }
    return *this;
  }

  /** Move constructor */
  PrimalSolution(PrimalSolution&& other) noexcept = default;

  /** Move Assignment */
  PrimalSolution& operator=(PrimalSolution&& other) noexcept = default;

  /** Swap */
  void swap(PrimalSolution& other) {
    timeTrajectory_.swap(other.timeTrajectory_);
    stateTrajectory_.swap(other.stateTrajectory_);
    inputTrajectory_.swap(other.inputTrajectory_);
    postEventIndices_.swap(other.postEventIndices_);
    ::ocs2::swap(modeSchedule_, other.modeSchedule_);
    controllerPtr_.swap(other.controllerPtr_);
  }

  void clear() {
    timeTrajectory_.clear();
    stateTrajectory_.clear();
    inputTrajectory_.clear();
    postEventIndices_.clear();
    modeSchedule_.clear();
    if (controllerPtr_ != nullptr) {
      controllerPtr_->clear();
    }
  }

  scalar_array_t timeTrajectory_;
  vector_array_t stateTrajectory_;
  vector_array_t inputTrajectory_;
  size_array_t postEventIndices_;
  ModeSchedule modeSchedule_;
  std::unique_ptr<ControllerBase> controllerPtr_;
};

}  // namespace ocs2
