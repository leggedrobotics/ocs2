

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
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @param [in] x: a floating-point number.
 * @param [in] y: a floating-point number.
 * @return bool: true if x=y.
 */
template <class T1, class T2>
typename std::enable_if<std::is_floating_point<typename std::remove_reference<T1>::type>::value ||
                            std::is_floating_point<typename std::remove_reference<T2>::type>::value,
                        bool>::type
almost_eq(T1&& x, T2&& y) {
  typedef typename std::conditional<std::is_floating_point<typename std::remove_reference<T1>::type>::value,
                                    typename std::remove_reference<T1>::type, typename std::remove_reference<T2>::type>::type TypeResult;
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::abs(x - y) <= std::numeric_limits<TypeResult>::epsilon() * std::abs(x + y)
         // unless the result is subnormal
         || std::abs(x - y) < std::numeric_limits<TypeResult>::min();
}

/**
 * Almost less-equal which uses machine epsilon to compare floating-point values for equality.
 *
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @param [in] x: a floating-point number.
 * @param [in] y: a floating-point number.
 * @return bool: true if x<=y.
 */
template <class T1, class T2>
typename std::enable_if<std::is_floating_point<typename std::remove_reference<T1>::type>::value ||
                            std::is_floating_point<typename std::remove_reference<T2>::type>::value,
                        bool>::type
almost_le(T1&& x, T2&& y) {
  return x < y || almost_eq(x, y);
}

/**
 * Almost greater-equal which uses machine epsilon to compare floating-point values for equality.
 *
 * @tparam T1: data type of x.
 * @tparam T2: data type of y.
 * @param [in] x: a floating-point number.
 * @param [in] y: a floating-point number.
 * @return bool: true if x>=y.
 */
template <class T1, class T2>
typename std::enable_if<std::is_floating_point<typename std::remove_reference<T1>::type>::value ||
                            std::is_floating_point<typename std::remove_reference<T2>::type>::value,
                        bool>::type
almost_ge(T1&& x, T2&& y) {
  return x > y || almost_eq(x, y);
}

}  // namespace numerics
}  // namespace ocs2

#endif  // OCS2_NUMERICS_H
