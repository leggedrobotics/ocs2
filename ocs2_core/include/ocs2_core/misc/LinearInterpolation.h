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
 * Default subfield access function, returns value itself.
 */
template <typename T>
struct Identity {
  const T& operator()(const T& value) const { return value; };
};

/**
 * Get the interval index and interpolation coefficient alpha.
 * Alpha = 1 at the start of the interval and alpha = 0 at the end.
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [in] timeArrayPtr: interpolation time array.
 * @return {index, alpha}
 */
inline index_alpha_t timeSegment(scalar_t enquiryTime, const std::vector<scalar_t>* timeArrayPtr) {
  // corner cases (no time set OR single time element)
  if (timeArrayPtr == nullptr || timeArrayPtr->size() <= 1) {
    return {0, scalar_t(1.0)};
  }

  int index = lookup::findIntervalInTimeArray(*timeArrayPtr, enquiryTime);
  auto lastInterval = static_cast<int>(timeArrayPtr->size() - 1);
  if (index >= 0) {
    if (index < lastInterval) {
      // interpolation : 0 <= index < lastInterval
      scalar_t alpha = (enquiryTime - (*timeArrayPtr)[index + 1]) / ((*timeArrayPtr)[index] - (*timeArrayPtr)[index + 1]);
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
 *  - No data (nullptrs or zero size containers) implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair
 * @param [out] enquiryData : result of the interpolation
 * @param [in] dataPtr: Pointer to vector of data
 *
 * @tparam AccessFun: Subfield data access functor
 * @tparam Data: Data type
 * @tparam Field: Data's subfield type.
 * @tparam Alloc: Specialized allocation class
 */
template <class AccessFun, typename Data, typename Field, class Alloc>
void interpolateField(index_alpha_t indexAlpha, Field& enquiryData, const std::vector<Data, Alloc>* dataPtr) {
  if (dataPtr != nullptr) {
    AccessFun accessFun;
    if (dataPtr->size() > 1) {
      // Normal interpolation case
      int index = indexAlpha.first;
      scalar_t alpha = indexAlpha.second;
      auto& lhs = accessFun((*dataPtr)[index]);
      auto& rhs = accessFun((*dataPtr)[index + 1]);
      if (areSameSize(rhs, lhs)) {
        enquiryData = alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
      } else {
        enquiryData = (alpha > 0.5) ? lhs : rhs;
      }
    } else if (dataPtr->size() == 1) {
      // Time vector has only 1 element -> Constant function
      enquiryData = accessFun((*dataPtr)[0]);
    } else {
      // Time empty -> zero function
      enquiryData *= scalar_t(0.0);
    }
  } else {
    // No data set -> zero Function
    enquiryData *= scalar_t(0.0);
  }
}

/**
 * Linearly interpolates at the given time. When duplicate values exist the lower range is selected s.t. ( ]
 * Example: t = [0.0, 1.0, 1.0, 2.0]
 * when querying tk = 1.0, the range (0.0, 1.0] is selected
 *
 *  - No data (nullptrs or zero size containers) implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [out] enquiryData: The value of the trajectory at the requested time.
 * @param [in] timeStampPtr: Pointer to vector of times
 * @param [in] dataPtr: Pointer to vector of data
 * @return {index, alpha}: The greatest smaller time stamp index and the interpolation coefficient [1, 0]
 *
 * @tparam AccessFun: Subfield data access functor
 * @tparam Data: Data type
 * @tparam Field: Data's subfield type.
 * @tparam Alloc: Specialized allocation class
 */
template <class AccessFun, typename Data, typename Field, class Alloc>
index_alpha_t interpolateField(scalar_t enquiryTime, Field& enquiryData, const std::vector<scalar_t>* timeStampPtr,
                               const std::vector<Data, Alloc>* dataPtr) {
  auto indexAlpha = timeSegment(enquiryTime, timeStampPtr);
  interpolateField<AccessFun, Data, Field, Alloc>(indexAlpha, enquiryData, dataPtr);
  return indexAlpha;
}

/**
 * Same as interpolateField(indexAlpha, enquiryData, dataPtr)
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair
 * @param [out] enquiryData : result of the interpolation
 * @param [in] dataPtr: Pointer to vector of data
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc>
void interpolate(index_alpha_t indexAlpha, Data& enquiryData, const std::vector<Data, Alloc>* dataPtr) {
  interpolateField<Identity<Data>>(indexAlpha, enquiryData, dataPtr);
}

/**
 * Same as interpolateField(enquiryTime, enquiryData, timeStampPtr, dataPtr)
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [out] enquiryData: The value of the trajectory at the requested time.
 * @param [in] timeStampPtr: Pointer to vector of times
 * @param [in] dataPtr: Pointer to vector of data
 * @return {index, alpha}: The greatest smaller time stamp index and the interpolation coefficient [1, 0]
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc>
index_alpha_t interpolate(scalar_t enquiryTime, Data& enquiryData, const std::vector<scalar_t>* timeStampPtr,
                          const std::vector<Data, Alloc>* dataPtr) {
  return interpolateField<Identity<Data>>(enquiryTime, enquiryData, timeStampPtr, dataPtr);
}

}  // namespace LinearInterpolation
}  // namespace ocs2
