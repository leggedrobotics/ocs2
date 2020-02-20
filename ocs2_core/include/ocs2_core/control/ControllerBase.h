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

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/control/ControllerType.h"

namespace ocs2 {

/**
 * The base class for all controllers.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ControllerBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dimensions_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimensions_t::scalar_t;
  using size_array_t = typename dimensions_t::size_array_t;
  using scalar_array_t = typename dimensions_t::scalar_array_t;
  using float_array_t = std::vector<float>;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using input_vector_t = typename dimensions_t::input_vector_t;

  using self_t = ControllerBase<STATE_DIM, INPUT_DIM>;
  using array_t = std::vector<self_t, Eigen::aligned_allocator<self_t>>;

  /**
   * Default constructor.
   */
  ControllerBase() = default;

  /**
   * Default destructor.
   */
  virtual ~ControllerBase() = default;

  /**
   * @brief Computes the control command at a given time and state.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return Current input.
   */
  virtual input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) = 0;

  /**
   * @brief Saves the controller at given time to an array of arrays structure for ROS transmission
   * @param[in] timeArray array of query times
   * @param[out] flatArray2 The array of arrays that is to be filled, i.e., the compressed controller. One array per query time
   */
  virtual void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const = 0;

  /**
   * @brief Restores and initializes the controller from a flattened array
   * @param[in] timeArray array of times
   * @param[in] flatArray2 The array the represents the compressed controller
   */
  virtual void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) = 0;

  /**
   * @brief Merges this controller with another controller that comes active later in time
   * This method is typically used to merge controllers from multiple time partitions.
   * Indices specifying a range of elements. Copies of the elements in the range [index, index_length) are inserted at the end.
   *
   * @note Only controllers of the same type can be merged.
   *
   * @param[in] otherController: The control law to be appended.
   * @param[in] index: The starting index
   * @param[in] length: The length of the copy.
   */
  virtual void concatenate(const ControllerBase* otherController, int index, int length) = 0;

  /**
   * @brief Merges this controller with another controller that comes active later in time
   * This method is typically used to merge controllers from multiple time partitions.
   * @note Only controllers of the same type can be merged.
   *
   * @param[in] otherController: The control law to be appended.
   */
  void concatenate(const ControllerBase* otherController) { concatenate(otherController, 0, otherController->size()); }

  /**
   * @brief Returns the size of the controller.
   *
   * @return The size of the controller.
   */
  virtual int size() const = 0;

  /**
   * @brief Prints the type of controller
   * @return ControllerType: what type of controller this is
   */
  virtual ControllerType getType() const = 0;

  /**
   * @brief clears and reverts back to an empty controller.
   * Therefore, if empty() method is called, it will return true.
   */
  virtual void clear() = 0;

  /**
   * @brief Fills all the data containers with zeros. Does not change size, does not change time array.
   */
  virtual void setZero() = 0;

  /**
   * Returns whether the class contains any information.
   *
   * @return true if it contains no information, false otherwise.
   */
  virtual bool empty() const = 0;

  /**
   * @brief Create a deep copy of the object.
   * @warning Cloning implies that the caller takes ownership and deletes the created object.
   * @return Pointer to a new instance.
   */
  virtual ControllerBase* clone() const = 0;

  /**
   * Displays controller's data.
   */
  virtual void display() const {}

  /**
   * @brief Gets the event times for which the controller is designed.
   * @return The event times of the controller.
   */
  virtual scalar_array_t controllerEventTimes() const {}
};

}  // namespace ocs2
