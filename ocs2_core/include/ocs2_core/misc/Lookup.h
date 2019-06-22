//
// Created by rgrandia on 20.06.19.
//

#ifndef OCS2_CTRL_LOOKUP_H
#define OCS2_CTRL_LOOKUP_H

#include <vector>
#include <complex>

#include "ocs2_core/OCS2NumericTraits.h"

namespace ocs2 {
namespace Lookup {

/**
 * finds the index of an element in a sorted dataArray which is equal to value (epsilone distance)
 * @param [in] dataArray: data array
 * @param [in] value: enquiry value
 * @return: index
 */
template <typename scalar_t = double>
size_t findFirstIndexWithinTol(
		const std::vector<scalar_t> &dataArray,
		const scalar_t &value,
		const scalar_t eps = OCS2NumericTraits<scalar_t>::week_epsilon()) {

  size_t index = dataArray.size();

  for (size_t i = 0; i < dataArray.size(); i++){
    if (std::abs(dataArray[i] - value) < eps) {
      index = i;
      break;
    }
  }

  if (index == dataArray.size()) {
    throw std::runtime_error("[findIndexOfClosestWithTol] Value not found within tolerance");
  }

  return index;
}

} // namespace Lookup
} // namespace ocs2

#endif //OCS2_CTRL_LOOKUP_H
