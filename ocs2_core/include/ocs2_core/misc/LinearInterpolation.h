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

#include "ocs2_core/NumericTraits.h"
#include "ocs2_core/Types.h"
#include "ocs2_core/misc/Lookup.h"

namespace ocs2 {
namespace LinearInterpolation {

using index_alpha_t = std::pair<int, scalar_t>;

template <class T>
using remove_cvref_t = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

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
 * Helper access function for std::vector<Data, Alloc> where Data a simple data structure such as double, Eigen-Type.
 */
template <typename Data, class Alloc>
const Data& stdAccessFun(const std::vector<Data, Alloc>& vec, size_t ind) {
  return vec[ind];
}

/**
 * Get the interval index and interpolation coefficient alpha.
 * Alpha = 1 at the start of the interval and alpha = 0 at the end.
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [in] timeArray: interpolation time array.
 * @return {index, alpha}
 */
inline index_alpha_t timeSegment(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray) {
  // corner cases (no time set OR single time element)
  if (timeArray.size() <= 1) {
    return {0, scalar_t(1.0)};
  }

  const int index = lookup::findIntervalInTimeArray(timeArray, enquiryTime);
  const auto lastInterval = static_cast<int>(timeArray.size() - 1);
  if (index >= 0) {
    if (index < lastInterval) {
      // interpolation : 0 <= index < lastInterval
      assert(enquiryTime <= timeArray[index + 1]);  // assert upper bound of lookup
      assert(timeArray[index] <= enquiryTime);      // assert lower bound of lookup
      const scalar_t intervalLength = timeArray[index + 1] - timeArray[index];
      const scalar_t timeTillNext = timeArray[index + 1] - enquiryTime;

      // Normal case: interval is large enough for normal interpolation
      constexpr scalar_t minIntervalTime = 2.0 * numeric_traits::weakEpsilon<scalar_t>();
      if (intervalLength > minIntervalTime) {
        return {index, (timeTillNext / intervalLength)};
      }

      // Take closes point for small time intervals
      if (timeTillNext < 0.5 * intervalLength) {  // short interval, closest to time[index + 1]
        return {index, scalar_t(0.0)};
      } else {  // short interval, closest to time[index]
        return {index, scalar_t(1.0)};
      }
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
 * @param [in] dataArray: vector of data
 * @param [in] accessFun: Method to access the subfield of Data in vector
 *
 * @tparam Data: Data type
 * @tparam Field: Data's subfield type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc, class AccessFun>
auto interpolate(index_alpha_t indexAlpha, const std::vector<Data, Alloc>& dataArray, AccessFun accessFun)
    -> remove_cvref_t<typename std::result_of<AccessFun(const std::vector<Data, Alloc>&, size_t)>::type> {
  assert(dataArray.size() > 0);
  if (dataArray.size() > 1) {
    // Normal interpolation case
    int index = indexAlpha.first;
    scalar_t alpha = indexAlpha.second;
    auto& lhs = accessFun(dataArray, index);
    auto& rhs = accessFun(dataArray, index + 1);
    if (areSameSize(rhs, lhs)) {
      return alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
    } else {
      return (alpha > 0.5) ? lhs : rhs;
    }
  } else {  // dataArray.size() == 1
    // Time vector has only 1 element -> Constant function
    return accessFun(dataArray, 0);
  }
}

/** Default interpolation */
template <typename Data, class Alloc>
Data interpolate(index_alpha_t indexAlpha, const std::vector<Data, Alloc>& dataArray) {
  return interpolate(indexAlpha, dataArray, stdAccessFun<Data, Alloc>);
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
 * @param [in] timeArray: Times vector
 * @param [in] dataArray: Data vector
 * @param [in] accessFun: Subfield data access method
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Field: Data's subfield type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, typename Field, class Alloc>
Field interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray, const std::vector<Data, Alloc>& dataArray,
                  const Field& (*accessFun)(const std::vector<Data, Alloc>&, size_t)) {
  return interpolate(timeSegment(enquiryTime, timeArray), dataArray, accessFun);
}

/** Default interpolation */
template <typename Data, class Alloc>
Data interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray, const std::vector<Data, Alloc>& dataArray) {
  return interpolate<Data, Data, Alloc>(enquiryTime, timeArray, dataArray, stdAccessFun<Data, Alloc>);
}

}  // namespace LinearInterpolation
}  // namespace ocs2
