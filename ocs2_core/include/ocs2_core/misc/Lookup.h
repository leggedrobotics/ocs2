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

#include <algorithm>
#include <sstream>
#include <vector>

#include <ocs2_core/NumericTraits.h>
#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {
namespace lookup {

/**
 * finds the index of an element in a sorted dataArray which is equal to value (epsilon distance)
 *
 * @tparam SCALAR : scalar type
 * @param [in] dataArray: data array
 * @param [in] value: enquiry value
 * @return: index
 */
template <typename SCALAR = double>
size_t findFirstIndexWithinTol(const std::vector<SCALAR>& dataArray, SCALAR value, SCALAR eps = numeric_traits::weakEpsilon<SCALAR>()) {
  // Search for a match by linearly traversing the data, returning first match
  for (size_t i = 0; i < dataArray.size(); i++) {
    if (std::abs(dataArray[i] - value) < eps) {
      return i;
    }
  }

  // If we reach here, no match was found
  std::stringstream msg;
  msg << "[findFirstIndexWithinTol] Value not found within tolerance, with \n\t dataArray: ";
  for (const auto d : dataArray) {
    msg << d << ", ";
  }
  msg << "\n\t value: " << value << "\n\t epsilon: " << eps << std::endl;
  throw std::runtime_error(msg.str());
}

/**
 *  Find index into a sorted time Array
 *
 *  Indices are counted as follows:
 *          ------ | ----- | ---  ... ---    | -----
 *                t0     t1              t(n-1)
 *  Index     0        1      2   ...  (n-1)    n
 *
 *  Corner cases:
 *     - If time equal to a time in the timeArray is requested, the lower index is taken (e.g. t = t1 -> index = 1)
 *     - If multiple times in the timeArray are equal, the index before the first occurrence is taken.
 *       for example: if t1 = t2 = t3  and the requested time t <= t3 -> index = 1
 *
 *
 * @tparam SCALAR : numerical type of time
 * @param timeArray : sorted time array to perform the lookup in
 * @param time : enquiry time
 * @return index between [0, size(timeArray)]
 */
template <typename SCALAR = double>
int findIndexInTimeArray(const std::vector<SCALAR>& timeArray, SCALAR time) {
  auto firstLargerValueIterator = std::lower_bound(timeArray.begin(), timeArray.end(), time);
  return static_cast<int>(firstLargerValueIterator - timeArray.begin());
}

/**
 *  Find interval into a sorted time Array
 *
 *  Intervals are counted as follows:
 *           ------ | ----- | ---  ... ---    | -----
 *                 t0     t1              t(n-1)
 *  Interval  -1       0      1   ...  (n-2)    (n-1)
 *
 *  Corner cases are handled as in findIndexInTimeArray
 *
 * @tparam SCALAR : numerical type of time
 * @param timeArray : sorted time array to perform the lookup in
 * @param time : enquiry time
 * @return interval between [-1, size(timeArray)-1]
 */
template <typename SCALAR = double>
int findIntervalInTimeArray(const std::vector<SCALAR>& timeArray, SCALAR time) {
  if (!timeArray.empty()) {
    return findIndexInTimeArray(timeArray, time) - 1;
  } else {
    return 0;
  }
}

/**
 * Same as findIntervalInTimeArray except for 1 rule:
 * if t = t0, a 0 is returned instead of -1
 * This means that any query t_front <= t <= t_back gets assigned to a partition [0, size -2]
 *
 * @tparam SCALAR : scalar type
 * @param timeArray : sorted time array to perform the lookup in
 * @param time : enquiry time
 * @return partition between [-1, size(timeArray)-1]
 */
template <typename SCALAR = double>
int findActiveIntervalInTimeArray(const std::vector<SCALAR>& timeArray, SCALAR time) {
  if (!timeArray.empty() && !numerics::almost_eq(time, timeArray.front())) {
    return findIntervalInTimeArray(timeArray, time);
  } else {  // t = t0
    return 0;
  }
}

/**
 * Wraps findActiveIntervalInTimeArray with bound check
 * throws an error if time before of after the end is selected.
 * Also throws on timeArrays that are empty or with 1 element only.
 *
 * @tparam SCALAR : scalar type
 * @param timeArray : sorted time array to perform the lookup in
 * @param time : enquiry time
 * @return partition between [0, size(timeArray)-2]
 */
template <typename SCALAR = double>
int findBoundedActiveIntervalInTimeArray(const std::vector<SCALAR>& timeArray, SCALAR time) {
  auto partition = findActiveIntervalInTimeArray(timeArray, time);

  if (timeArray.empty()) {
    std::string mesg = "[findBoundedActiveIntervalInTimeArray] Time array is empty";
    throw std::runtime_error(mesg);
  }

  if (partition < 0) {
    std::string mesg =
        "[findBoundedActiveIntervalInTimeArray] Given time is less than the start time (i.e. givenTime < timeArray.front()): " +
        std::to_string(time) + " < " + std::to_string(timeArray.front());
    throw std::runtime_error(mesg);
  }

  if (partition >= timeArray.size() - 1) {
    std::string mesg =
        "[findBoundedActiveIntervalInTimeArray] Given time is greater than the final time (i.e. timeArray.back() < givenTime): " +
        std::to_string(timeArray.back()) + " < " + std::to_string(time);
    throw std::runtime_error(mesg);
  }

  return partition;
}

}  // namespace lookup
}  // namespace ocs2
