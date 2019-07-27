//
// Created by rgrandia on 20.06.19.
//

#ifndef OCS2_CTRL_LOOKUP_H
#define OCS2_CTRL_LOOKUP_H

#include <sstream>
#include <vector>

#include "ocs2_core/OCS2NumericTraits.h"

namespace ocs2 {
namespace Lookup {

/**
 * finds the index of an element in a sorted dataArray which is equal to value (epsilon distance)
 * @param [in] dataArray: data array
 * @param [in] value: enquiry value
 * @return: index
 */
template <typename scalar_t = double>
size_t findFirstIndexWithinTol(const std::vector<scalar_t>& dataArray, scalar_t value,
                               scalar_t eps = OCS2NumericTraits<scalar_t>::weakEpsilon()) {
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
 *  		------ | ----- | ---  ... ---    | -----
 *				   t0     t1              t(n-1)
 *  Index     0        1      2   ...  (n-1)    n
 *
 *  Corner cases:
 *     - If time equal to a time in the timeArray is requested, the lower index is taken (e.g. t = t1 -> index = 1)
 *     - If multiple times in the timeArray are equal, the index before the first occurance is taken.
 *       for example: if t1 = t2 = t3  and the requested time t <= t3 -> index = 1
 *
 *
 * @tparam scalar_t : numerical type of time
 * @param timeArray : sorted time array to perform the lookup in
 * @param time : enquiry time
 * @return index between [0, size(timeArray)]
 */
template <typename scalar_t = double>
int findIndexInTimeArray(const std::vector<scalar_t>& timeArray,  scalar_t time) {
  auto firstLargerValueIterator = std::lower_bound(timeArray.begin(), timeArray.end(), time);
  return static_cast<int>(firstLargerValueIterator - timeArray.begin());
};

/**
 *  Find interval into a sorted time Array
 *
 *  Intervals are counted as follows:
 *  		------ | ----- | ---  ... ---    | -----
 *				   t0     t1              t(n-1)
 *  Interval  -1       0      1   ...  (n-2)    (n-1)
 *
 *  Corner cases are handled as in findIndexInTimeArray
 *
 * @tparam scalar_t : numerical type of time
 * @param timeArray : sorted time array to perform the lookup in
 * @param time : enquiry time
 * @return interval between [-1, size(timeArray)-1]
 */
template <typename scalar_t = double>
int findIntervalInTimeArray(const std::vector<scalar_t>& timeArray,  scalar_t time) {
  if (!timeArray.empty()){
    return findIndexInTimeArray(timeArray, time) - 1;
  } else {
    return 0;
  }
};

/**
 *  Same as findIntervalInTimeArray except for 1 rule:
 *  if t = t0, a 0 is returned instead of -1
 *  This means that any query t_front <= t <= t_back gets assigned to a partition [0, size -2]
 *
 *  @return partition between [-1, size(timeArray)-1]
 */
template <typename scalar_t = double>
int findPartitionInTimeArray(const std::vector<scalar_t>& timeArray,  scalar_t time) {
  if (!timeArray.empty() && time != timeArray.front()) {
    return findIntervalInTimeArray(timeArray, time);
  } else { // t = t0
    return 0;
  }
};

/**
 * Wraps findPartitionInTimeArray with bound check
 * throws an error if time before of after the end is selected.
 * Also throws on timeArrays that are empty or with 1 element only.
 *
 * @return partition between [0, size(timeArray)-2]
 */
template <typename scalar_t = double>
int findActivePartitionInTimeArray(const std::vector<scalar_t>& timeArray,  scalar_t time) {
  auto partition = findPartitionInTimeArray(timeArray, time);

  if (timeArray.empty()) {
    std::string mesg = "[findActivePartitionInTimeArray] Time array is empty";
    throw std::runtime_error(mesg);
  }

  if (partition < 0) {
    std::string mesg = "[findActivePartitionInTimeArray] Given time is less than the start time (i.e. givenTime < timeArray.front()): " + std::to_string(time) +
        " < " + std::to_string(timeArray.front());
    throw std::runtime_error(mesg);
  }

  if (partition >= timeArray.size() - 1) {
    std::string mesg = "[findActivePartitionInTimeArray] Given time is greater than the final time (i.e. timeArray.back() < givenTime): " +
        std::to_string(timeArray.back()) + " < " + std::to_string(time);
    throw std::runtime_error(mesg);
  }

  return partition;
};


}  // namespace Lookup
}  // namespace ocs2

#endif  // OCS2_CTRL_LOOKUP_H
