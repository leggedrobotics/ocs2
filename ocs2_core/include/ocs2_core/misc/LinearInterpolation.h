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

#include <type_traits>
#include <utility>
#include <vector>

#include "ocs2_core/Types.h"

namespace ocs2 {
namespace LinearInterpolation {

using index_alpha_t = std::pair<int, scalar_t>;

template <class T>
using remove_cvref_t = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

/**
 * Get the interval index and interpolation coefficient alpha.
 * Alpha = 1 at the start of the interval and alpha = 0 at the end.
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [in] timeArray: interpolation time array.
 * @return {index, alpha}
 */
index_alpha_t timeSegment(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray);

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
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc>
Data interpolate(index_alpha_t indexAlpha, const std::vector<Data, Alloc>& dataArray);

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
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc>
Data interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray, const std::vector<Data, Alloc>& dataArray);

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
 * @param [in] accessFun: Method to access the subfield of Data in array. The signature of the accessFun
 *                        should be equivalent to the following where Field is any subfield of Data:
 *                        const Field& AccessFun(const std::vector<Data, Alloc>& array, size_t index)
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc, class AccessFun>
auto interpolate(index_alpha_t indexAlpha, const std::vector<Data, Alloc>& dataArray, AccessFun accessFun)
    -> remove_cvref_t<typename std::result_of<AccessFun(const std::vector<Data, Alloc>&, size_t)>::type>;

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
 * @param [in] accessFun: Method to access the subfield of Data in array. The signature of the accessFun
 *                        should be equivalent to the following where Field is any subfield of Data:
 *                        const Field& AccessFun(const std::vector<Data, Alloc>& array, size_t index)
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc, class AccessFun>
auto interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray, const std::vector<Data, Alloc>& dataArray,
                 AccessFun accessFun) -> remove_cvref_t<typename std::result_of<AccessFun(const std::vector<Data, Alloc>&, size_t)>::type>;

}  // namespace LinearInterpolation
}  // namespace ocs2

#include "implementation/LinearInterpolation.h"
