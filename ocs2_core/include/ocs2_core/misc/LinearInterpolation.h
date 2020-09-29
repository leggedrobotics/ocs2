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

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <algorithm>
#include <functional>
#include <type_traits>
#include <utility>
#include <vector>

#include "ocs2_core/Types.h"
#include "ocs2_core/misc/Lookup.h"

namespace ocs2 {
namespace LinearInterpolation {

using index_alpha_t = std::pair<int, scalar_t>;

/**
 * Helper comparison function for non-Eigen types.
 */
template <typename T>
constexpr typename std::enable_if<!std::is_base_of<Eigen::EigenBase<T>, T>::value, bool>::type areSameSize(const T& lhs, const T& rhs) {
  return true;
}

/**
 * Helper comparison function for Eigen-Type.
 */
template <typename Derived>
bool areSameSize(const Eigen::EigenBase<Derived>& lhs, const Eigen::EigenBase<Derived>& rhs) {
  return lhs.rows() == rhs.rows() && lhs.cols() == rhs.cols();
}

/**
 * Get the interval index and interpolation coefficient alpha.
 * Alpha = 1 at the start of the interval and alpha = 0 at the end.
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [in] times: interpolation time array.
 * @return {index, alpha}
 */
inline index_alpha_t timeSegment(scalar_t enquiryTime, const std::vector<scalar_t>& times) {
  // corner cases (no time set OR single time element)
  if (times.size() <= 1) {
    return {0, scalar_t(1.0)};
  }

  int index = lookup::findIntervalInTimeArray(times, enquiryTime);
  auto lastInterval = static_cast<int>(times.size() - 1);
  if (index >= 0) {
    if (index < lastInterval) {
      // interpolation : 0 <= index < lastInterval
      scalar_t alpha = (enquiryTime - times[index + 1]) / (times[index] - times[index + 1]);
      return {index, alpha};
    } else {
      // upper bound : index >= lastInterval
      return {std::max(lastInterval - 1, 0), scalar_t(0.0)};
    }
  } else {
    // lower bound : index < 0
    return {0, scalar_t(1.0)};
  }
}

/**
 * Directly uses the index and interpolation coefficient provided by the user
 * @note If sizes in data array are not equal, the interpolation will snap to the data
 * point closest to the query time
 *
 *  - No data implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair
 * @param [in] data: vector of data, typically std::vector
 * @param [in] get: Subfield data access method
 * @return The interpolation result
 *
 * @tparam Container: Container type
 * @tparam Func: Subfield data access method
 */
template <typename Container, typename Func>
auto interpolate(index_alpha_t indexAlpha, const Container& data, Func get) -> decltype(get(data[0])) {
  assert(data.size() > 0);
  if (data.size() > 1) {
    // Normal interpolation case
    int index = indexAlpha.first;
    scalar_t alpha = indexAlpha.second;
    const auto lhs = get(data[index]);
    const auto rhs = get(data[index + 1]);
    if (areSameSize(rhs, lhs)) {
      return alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
    } else {
      return (alpha > 0.5) ? lhs : rhs;
    }
  } else {
    // Time vector has only 1 element -> Constant function
    return get(data[0]);
  }
}

/** Default interpolation */
template <typename Container>
auto interpolate(index_alpha_t indexAlpha, const Container& data) -> typename Container::value_type {
  return interpolate(indexAlpha, data, [](const typename Container::value_type& t) { return t; });
}

/**
 * Linearly interpolates at the given time. When duplicate values exist the lower range is selected s.t. ( ]
 * Example: t = [0.0, 1.0, 1.0, 2.0]
 * when querying tk = 1.0, the range (0.0, 1.0] is selected
 *
 *  - No data implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [in] times: Times vector
 * @param [in] data: Data vector, typically std::vector
 * @param [in] get: Subfield data access method
 * @return The interpolation result
 *
 * @tparam Container: Container type
 * @tparam Func: Subfield data access method
 */
template <typename Container, typename Func>
auto interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& times, const Container& data, Func get) -> decltype(get(data[0])) {
  return interpolate(timeSegment(enquiryTime, times), data, get);
}

/** Default interpolation */
template <typename Container>
auto interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& times, const Container& data) -> typename Container::value_type {
  return interpolate(enquiryTime, times, data, [](const typename Container::value_type& t) { return t; });
}

}  // namespace LinearInterpolation
}  // namespace ocs2
