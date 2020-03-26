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

#ifndef SYSTEMOPERATINGTRAJECTORIESBASE_OCS2_H_
#define SYSTEMOPERATINGTRAJECTORIESBASE_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <type_traits>
#include <vector>

#include "ocs2_core/Dimensions.h"

namespace ocs2 {

/**
 * This is base class for initializing the SLQ-based algorithms.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class SystemOperatingTrajectoriesBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM> >;
  using ConstPtr = std::shared_ptr<const SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM> >;
  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;

  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;

  /**
   * Default constructor
   */
  SystemOperatingTrajectoriesBase() = default;

  /**
   * Default destructor.
   */
  virtual ~SystemOperatingTrajectoriesBase() = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM>* clone() const = 0;

  /**
   * Gets the Operating Trajectories of the system in time interval [startTime, finalTime] where there is
   * no intermediate switches except possibly the end time.
   *
   * @param [in] initialState: Initial state.
   * @param [in] startTime: Initial time.
   * @param [in] finalTime: Final time.
   * @param [out] timeTrajectory: Output time stamp trajectory.
   * @param [out] stateTrajectory: Output state trajectory.
   * @param [out] inputTrajectory: Output control input trajectory.
   * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or
   * override (default).
   */
  virtual void getSystemOperatingTrajectories(const state_vector_t& initialState, const scalar_t& startTime, const scalar_t& finalTime,
                                              scalar_array_t& timeTrajectory, state_vector_array_t& stateTrajectory,
                                              input_vector_array_t& inputTrajectory, bool concatOutput = false) = 0;

 private:
};

}  // namespace ocs2

#endif /* SYSTEMOPERATINGTRAJECTORIESBASE_OCS2_H_ */
