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

#ifndef LINEARINTERPOLATION_OCS2_H_
#define LINEARINTERPOLATION_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <algorithm>
#include <memory>
#include <vector>

#include <ocs2_core/misc/Lookup.h>

namespace ocs2 {

/**
 * Linear Interpolation class.
 *  - No data (nullptrs or zero size containers) implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are user for linear interpolation and zero order extrapolation
 *
 * @tparam Data_T: Date type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data_T, class Alloc = std::allocator<Data_T>>
class LinearInterpolation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = double;

  /**
   * Default constructor.
   */
  LinearInterpolation() = default;

  /**
   * Default destructor.
   */
  ~LinearInterpolation() = default;

  /**
   * Linearly interpolates at the given time. When duplicate values exist the lower range is selected s.t. ( ]
   * Example: t = [0.0, 1.0, 1.0, 2.0]
   * when querying tk = 1.0, the range (0.0, 1.0] is selected
   *
   * @param [in]  enquiryTime: The enquiry time for interpolation.
   * @param [out] enquiryData: The value of the trajectory at the requested time.
   * @param [in] timeStampPtr: Pointer to vector of times
   * @param [in] dataPtr: Pointer to vector of data
   * @return {index, alpha}: The greatest smaller time stamp index and the interpolation coefficient [1, 0]
   */
  template <typename Field_T>
  static std::pair<int, scalar_t> interpolate(scalar_t enquiryTime, Field_T& enquiryData, const std::vector<scalar_t>* timeStampPtr,
                                              const std::vector<Data_T, Alloc>* dataPtr) {
    const auto indexAlpha = timeSegment(enquiryTime, timeStampPtr);
    interpolate(indexAlpha, enquiryData, dataPtr);
    return indexAlpha;
  }

  /**
   * Directly uses the index and interpolation coefficient provided by the user
   * @note If sizes in data array are not equal, the interpolation will snap to the data point closest to the query time
   *
   * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair
   * @param [out] enquiryData : result of the interpolation
   * @param [in] dataPtr: Pointer to vector of data
   */
  template <typename Field_T>
  static void interpolate(std::pair<int, scalar_t> indexAlpha, Field_T& enquiryData, const std::vector<Data_T, Alloc>* dataPtr) {
    if (dataPtr) {
      if (dataPtr->size() > 1) {
        // Normal interpolation case
        int index = indexAlpha.first;
        scalar_t alpha = indexAlpha.second;
        auto& lhs = (*dataPtr)[index];
        auto& rhs = (*dataPtr)[index + 1];
        if (lhs.rows() == rhs.rows() && lhs.cols() == rhs.cols()) {
          enquiryData = alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
        } else {
          enquiryData = (alpha > 0.5) ? lhs : rhs;
        }
      } else if (dataPtr->size() == 1) {
        // Time vector has only 1 element -> Constant function
        enquiryData = dataPtr->front();
      } else {
        // Time empty -> zero function
        enquiryData.setZero();
      }
    } else {
      // No data set -> zero Function
      enquiryData.setZero();
    }
  }

  /*
   * Specialization of interpolate() member method for floating point types.
   */
  static void interpolate(std::pair<int, scalar_t> indexAlpha, double& enquiryData, const std::vector<Data_T, Alloc>* dataPtr) {
    if (dataPtr) {
      if (dataPtr->size() > 1) {
        // Normal interpolation case
        int index = indexAlpha.first;
        scalar_t alpha = indexAlpha.second;
        auto& lhs = (*dataPtr)[index];
        auto& rhs = (*dataPtr)[index + 1];
        enquiryData = alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
      } else if (dataPtr->size() == 1) {
        // Time vector has only 1 element -> Constant function
        enquiryData = dataPtr->front();
      } else {
        // Time empty -> zero function
        enquiryData = scalar_t(0.0);
      }
    } else {
      // No data set -> zero Function
      enquiryData = scalar_t(0.0);
    }
  };

  /**
   * Get the interval index and interpolation coefficient alpha.
   * Alpha = 1 at the start of the interval and alpha = 0 at the end.
   *
   * @param [in] enquiryTime: The enquiry time for interpolation.
   * @param [in] timeArrayPtr: interpolation time array.
   * @return std::pair<int, double> : {index, alpha}
   */
  static std::pair<int, double> timeSegment(scalar_t enquiryTime, const std::vector<scalar_t>* timeArrayPtr) {
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
};

// Specialization for Eigen types
template <typename Data_T>
using EigenLinearInterpolation = LinearInterpolation<Data_T, Eigen::aligned_allocator<Data_T>>;

}  // namespace ocs2

#endif /* LINEARINTERPOLATION_H_ */
