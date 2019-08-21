

#ifndef OCS2_NUMERICS_H
#define OCS2_NUMERICS_H

#include <cmath>
#include <limits>
#include <type_traits>

namespace ocs2 {
namespace numerics {

/**
 * Almost equal which uses machine epsilon to compare floating-point values for equality.
 * refer to: https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
 *
 * @tparam T: data type.
 * @param [in] x: a floating-point number.
 * @param [in] y: a floating-point number.
 * @return bool: true if x=y.
 */
template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_eq(const T& x, const T& y) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::abs(x - y) <= std::numeric_limits<T>::epsilon() * std::abs(x + y)
         // unless the result is subnormal
         || std::abs(x - y) < std::numeric_limits<T>::min();
}

/**
 * Almost less-equal which uses machine epsilon to compare floating-point values for equality.
 *
 * @tparam T: data type.
 * @param [in] x: a floating-point number.
 * @param [in] y: a floating-point number.
 * @return bool: true if x<=y.
 */
template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_le(const T& x, const T& y) {
  return x < y || almost_eq(x, y);
}

/**
 * Almost greater-equal which uses machine epsilon to compare floating-point values for equality.
 *
 * @tparam T: data type.
 * @param [in] x: a floating-point number.
 * @param [in] y: a floating-point number.
 * @return bool: true if x>=y.
 */
template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_ge(const T& x, const T& y) {
  return x > y || almost_eq(x, y);
}

}  // namespace numerics
}  // namespace ocs2

#endif  // OCS2_NUMERICS_H
