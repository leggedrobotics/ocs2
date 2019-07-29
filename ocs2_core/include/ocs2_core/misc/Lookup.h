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

}  // namespace Lookup
}  // namespace ocs2

#endif  // OCS2_CTRL_LOOKUP_H
