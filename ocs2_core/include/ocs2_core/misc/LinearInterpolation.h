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
  LinearInterpolation() : timeStampPtr_(nullptr), dataPtr_(nullptr) {}

  /**
   * Constructor
   *
   * @param [in] timeStampPtr: A pointer to time stamp.
   * @param [in] dataPtr: A pointer to the data.
   */
  LinearInterpolation(const std::vector<scalar_t>* timeStampPtr, const std::vector<Data_T, Alloc>* dataPtr) {
    setData(timeStampPtr, dataPtr);
  }

  /**
   * Copy constructor
   *
   * @param [in] arg: Instance of the other class.
   */
  LinearInterpolation(const LinearInterpolation& arg) = default;

  /**
   * Sets the time stamp and data.
   *
   * @param [in] timeStampPtr: A pointer to time stamp.
   * @param [in] dataPtr: A pointer to the data.
   */
  void setData(const std::vector<scalar_t>* timeStampPtr, const std::vector<Data_T, Alloc>* dataPtr) {
    if (timeStampPtr != nullptr && dataPtr != nullptr) {
      if (dataPtr->size() != timeStampPtr->size()) {
        std::string errorMsg = "LinearInterpolation.h: sizes are not suitable for interpolation. TimeStamp has size " +
                               std::to_string(dataPtr->size()) + " but Data has size " + std::to_string(timeStampPtr->size()) + ".";
        throw std::runtime_error(errorMsg);
      }
    }
    timeStampPtr_ = timeStampPtr;
    dataPtr_ = dataPtr;
  }

  /**
   * Sets zero
   */
  void setZero() {
    timeStampPtr_ = nullptr;
    dataPtr_ = nullptr;
  }

  /**
   * Linearly interpolates at the given time. When duplicate values exist the lower range is selected s.t. ( ]
   * Example: t = [0.0, 1.0, 1.0, 2.0]
   * when querying tk = 1.0, the range (0.0, 1.0] is selected
   *
   * @param [in]  enquiryTime: The enquiry time for interpolation.
   * @param [out] enquiryData: The value of the trajectory at the requested time.
   * @return {index, alpha}: The greatest smaller time stamp index and the interpolation coefficient [1, 0]
   */
  std::pair<int, scalar_t> interpolate(const scalar_t& enquiryTime, Data_T& enquiryData) const {
    if (timeStampPtr_ != nullptr) {
      if (timeStampPtr_->size() > 1) {
        // Normal interpolation case, time vector has at least two elements
        const auto indexAlpha = getIndexAlpha(*timeStampPtr_, enquiryTime);
        interpolate(indexAlpha, enquiryData);
        return indexAlpha;
      } else if (timeStampPtr_->size() == 1) {
        // Time vector has only 1 element -> Constant function
        enquiryData = dataPtr_->front();
        return {0, scalar_t(0.0)};
      } else {
        // Time empty -> zero function
        enquiryData.setZero();
        return {0, scalar_t(0.0)};
      }
    } else {
      // No time set -> zero function
      enquiryData.setZero();
      return {0, scalar_t(0.0)};
    }
  }

  /**
   * Directly uses the index and interpolation coefficient provided by the user
   *
   * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair
   * @param [out] enquiryData : result of the interpolation
   */
  void interpolate(std::pair<int, scalar_t> indexAlpha, Data_T& enquiryData) const {
    if (dataPtr_ != nullptr) {
      if (dataPtr_->size() > 1) {
        // Normal interpolation case
        int index = indexAlpha.first;
        scalar_t alpha = indexAlpha.second;
        enquiryData = alpha * (*dataPtr_)[index] + (scalar_t(1.0) - alpha) * (*dataPtr_)[index + 1];
      } else if (dataPtr_->size() == 1) {
        // Time vector has only 1 element -> Constant function
        enquiryData = dataPtr_->front();
      } else {
        // Time empty -> zero function
        enquiryData.setZero();
      }
    } else {
      // No data set -> zero Function
      enquiryData.setZero();
    }
  }

 private:
  /**
   * Get the interval index and interpolation coefficient alpha.
   * Alpha = 1 at the start of the interval and alpha = 0 at the end.
   *
   * @param [in] timeArray: interpolation time array.
   * @param [in] enquiryTime: The enquiry time for interpolation.
   * @return std::pair<int, double> : {index, alpha}
   */
  static std::pair<int, double> getIndexAlpha(const std::vector<scalar_t>& timeArray, scalar_t enquiryTime) {
    int index = Lookup::findIntervalInTimeArray(timeArray, enquiryTime);
    auto lastInterval = static_cast<int>(timeArray.size() - 1);
    if (index >= 0) {
      if (index < lastInterval) {
        // interpolation : 0 <= index < lastInterval
        scalar_t alpha = (enquiryTime - timeArray[index + 1]) / (timeArray[index] - timeArray[index + 1]);
        return {index, alpha};
      } else {
        // upper bound : index >= lastInterval
        // Catch corner case of having timeArray.size() = 1 with max
        return {std::max(lastInterval - 1, 0), 0.0};
      }
    } else {
      // lower bound : index < 0
      return {0, 1.0};
    }
  }

  const std::vector<scalar_t>* timeStampPtr_;
  const std::vector<Data_T, Alloc>* dataPtr_;
};

// Specialization for Eigen types
template <typename Data_T>
using EigenLinearInterpolation = LinearInterpolation<Data_T, Eigen::aligned_allocator<Data_T>>;

}  // namespace ocs2

#endif /* LINEARINTERPOLATION_H_ */
