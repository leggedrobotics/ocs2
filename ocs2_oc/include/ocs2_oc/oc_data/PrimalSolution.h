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

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/logic/ModeSchedule.h>

namespace ocs2 {

/**
 * This class contains the primal problem's solution.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
struct PrimalSolution {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using controller_t = ControllerBase<STATE_DIM, INPUT_DIM>;
  using dim_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using size_array_t = typename dim_t::size_array_t;
  using scalar_array_t = typename dim_t::scalar_array_t;
  using state_vector_array_t = typename dim_t::state_vector_array_t;
  using input_vector_array_t = typename dim_t::input_vector_array_t;

  /**
   * Constructor
   */
  PrimalSolution() = default;

  /**
   * Destructor
   */
  ~PrimalSolution() = default;

  /**
   * Copy constructor
   */
  PrimalSolution(const PrimalSolution& other)
      : timeTrajectory_(other.timeTrajectory_),
        stateTrajectory_(other.stateTrajectory_),
        inputTrajectory_(other.inputTrajectory_),
        modeSchedule_(other.modeSchedule_),
        controllerPtr_(other.controllerPtr_->clone()) {}

  /**
   * Copy Assignment
   */
  PrimalSolution& operator=(const PrimalSolution& other) {
    timeTrajectory_ = other.timeTrajectory_;
    stateTrajectory_ = other.stateTrajectory_;
    inputTrajectory_ = other.inputTrajectory_;
    modeSchedule_ = other.modeSchedule_;
    controllerPtr_.reset(other.controllerPtr_->clone());
    return *this;
  }

  /**
   * Move constructor
   */
  PrimalSolution(PrimalSolution&& other) noexcept = default;

  /**
   * Move Assignement
   */
  PrimalSolution& operator=(PrimalSolution&& other) noexcept = default;

  scalar_array_t timeTrajectory_;
  state_vector_array_t stateTrajectory_;
  input_vector_array_t inputTrajectory_;
  ModeSchedule modeSchedule_;
  std::unique_ptr<controller_t> controllerPtr_;
};

}  // namespace ocs2
