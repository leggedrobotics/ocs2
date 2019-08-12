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

#ifndef TRAJECTORY_SPREADING_CONTROLLER_OCS2_H_
#define TRAJECTORY_SPREADING_CONTROLLER_OCS2_H_

#include <algorithm>
#include <iostream>
#include <utility>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/OCS2NumericTraits.h"
#include "ocs2_core/control/LinearController.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class TrajectorySpreadingController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { state_dim_ = STATE_DIM, input_dim_ = INPUT_DIM };

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;

  using controller_t = LinearController<STATE_DIM, INPUT_DIM>;
  using controller_array_t = typename controller_t::array_t;

  using index_t = std::pair<int, int>;  // (partition, index)

  /**
   * Constructor
   */
  TrajectorySpreadingController() = default;

  /**
   * Destructor
   */
  ~TrajectorySpreadingController() = default;

  /**
   * Adjust the controller based on the last changes in the logic rules.
   *
   * @param [in] eventTimes: The new event times.
   * @param [in] controllerEventTimes: The control policy stock's event times.
   * @param controllerStock: The controller stock which will be modified.
   */
  void adjustController(const scalar_array_t& eventTimes, const scalar_array_t& controllerEventTimes, controller_array_t& controllersStock);

 protected:
  /**
   * Finds the indices of the entries in the control policy that their time stamps are greater or equal to the associated event time.
   *
   * @param [in] eventTimes: Event time vector.
   * @param [in] controllersStock: Control policy.
   * @param [out] eventsIndices: event time indices in the control policy time stamp.
   */
  void findEventTimesIndices(const scalar_array_t& eventTimes, const controller_array_t& controllersStock,
                             std::vector<index_t>& eventsIndices) const;

  /**
   * Returns true if a <= b.
   *
   * @param [in] a: first index.
   * @param [in] b: second index.
   * @return true if a <= b.
   */
  bool isSmallerEqual(const index_t& a, const index_t& b) const;

  /**
   * Finds the index before the input index
   *
   * @param [in] a: first index.
   * @param [in] b: second index.
   * @return true if a <= b.
   */
  index_t findPreviousIndex(index_t index, const controller_array_t& controllersStock) const;

  /**
   *
   * @param eventTime
   * @param eventTimeIndex
   * @param ControlerEventTimeIndex
   * @param controllersStock
   */
  void spreadController(scalar_t eventTime, index_t eventTimeIndex, index_t controlerEventTimeIndex,
                        controller_array_t& controllersStock) const;

  /***********
   * Variables
   ***********/
  const int undefined_ = -1;
  int initActivePartitionIndex_ = 0;
  int finalActivePartitionIndex_ = 0;
};

}  // namespace ocs2

#include "implementation/TrajectorySpreadingController.h"

#endif /* TRAJECTORY_SPREADING_CONTROLLER_OCS2_H_ */
