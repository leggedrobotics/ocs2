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

#include <algorithm>
#include <functional>
#include <memory>
#include <type_traits>
#include <vector>

#include "ocs2_core/Dimensions.h"
#include "ocs2_core/misc/Lookup.h"

namespace ocs2 {

/**
 * Linear Interpolation class.
 *  - No data (nullptrs or zero size containers) implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are user for linear interpolation and zero order extrapolation
 */
class LinearInterpolation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = Dimensions<0, 0>::scalar_t;
  using dynamic_vector_t = Dimensions<0, 0>::dynamic_vector_t;
  using dynamic_matrix_t = Dimensions<0, 0>::dynamic_matrix_t;

  using size_type = typename std::vector<bool>::size_type;
  using index_alpha_t = std::pair<int, scalar_t>;

  /**
   * Get the interval index and interpolation coefficient alpha.
   * Alpha = 1 at the start of the interval and alpha = 0 at the end.
   *
   * @param [in] enquiryTime: The enquiry time for interpolation.
   * @param [in] timeArrayPtr: interpolation time array.
   * @return {index, alpha}
   */
  static index_alpha_t timeSegment(scalar_t enquiryTime, const std::vector<scalar_t>* timeArrayPtr) {
    // corner cases (no time set OR single time element)
    if (!timeArrayPtr || timeArrayPtr->size() <= 1) {
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
   * @note If sizes in data array are not equal, the interpolation will snap to the data point closest to the query time
   *
   * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair
   * @param [out] enquiryData : result of the interpolation
   * @param [in] dataPtr: Pointer to vector of data
   * @param accessFun: A method to access the subfield of DATA_T
   *
   * @tparam Data_T: Date type
   * @tparam Field_T: Data's subfield type.
   * @tparam Alloc: Specialized allocation class
   */
  template <typename Data_T, typename Field_T, class Alloc>
  static void interpolate(
      index_alpha_t indexAlpha, Field_T& enquiryData, const std::vector<Data_T, Alloc>* dataPtr,
      std::function<const Field_T&(const std::vector<Data_T, Alloc>*, size_type)> accessFun = stdAccessFun<Data_T, Alloc>) {
    if (dataPtr) {
      if (dataPtr->size() > 1) {
        // Normal interpolation case
        int index = indexAlpha.first;
        scalar_t alpha = indexAlpha.second;
        auto& lhs = accessFun(dataPtr, index);
        auto& rhs = accessFun(dataPtr, index + 1);
        if (compareSizes(rhs, lhs)) {
          enquiryData = alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
        } else {
          enquiryData = (alpha > 0.5) ? lhs : rhs;
        }
      } else if (dataPtr->size() == 1) {
        // Time vector has only 1 element -> Constant function
        enquiryData = accessFun(dataPtr, 0);
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
   * @param [in] enquiryTime: The enquiry time for interpolation.
   * @param [out] enquiryData: The value of the trajectory at the requested time.
   * @param [in] timeStampPtr: Pointer to vector of times
   * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair
   * @param [in] dataPtr: Pointer to vector of data
   * @param accessFun: A method to access the subfield of DATA_T
   * @return {index, alpha}: The greatest smaller time stamp index and the interpolation coefficient [1, 0]
   *
   * @tparam Data_T: Date type
   * @tparam Field_T: Data's subfield type.
   * @tparam Alloc: Specialized allocation class
   */
  template <typename Data_T, typename Field_T, class Alloc>
  static index_alpha_t interpolate(
      scalar_t enquiryTime, Field_T& enquiryData, const std::vector<scalar_t>* timeStampPtr, const std::vector<Data_T, Alloc>* dataPtr,
      std::function<const Field_T&(const std::vector<Data_T, Alloc>*, size_type)> accessFun = stdAccessFun<Data_T, Alloc>) {
    auto indexAlpha = timeSegment(enquiryTime, timeStampPtr);
    interpolate(indexAlpha, enquiryData, dataPtr, accessFun);
    return indexAlpha;
  }

  /**
   * Helper access function for std::vector<Data_T, Alloc> where Data_T a simple data structure such as double, Eigen-Type.
   */
  template <typename Data_T, class Alloc>
  static const Data_T& stdAccessFun(const std::vector<Data_T, Alloc>* vec, size_type ind) {
    return (*vec)[ind];
  }

 private:
  /**
   * Helper comparison function for non-Eigen types.
   */
  template <typename T>
  static typename std::enable_if<!std::is_base_of<Eigen::EigenBase<T>, T>::value, bool>::type compareSizes(const T& lhs, const T& rhs) {
    return true;
  }
  /**
   * Helper comparison function for Eigen-Type.
   */
  template <typename Derived>
  static bool compareSizes(const Eigen::EigenBase<Derived>& lhs, const Eigen::EigenBase<Derived>& rhs) {
    return lhs.rows() == rhs.rows() && lhs.cols() == rhs.cols();
  }
};
}  // namespace ocs2
