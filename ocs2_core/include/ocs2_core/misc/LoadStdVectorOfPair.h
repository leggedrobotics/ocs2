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

#include <string>
#include <utility>
#include <vector>

#include <boost/tokenizer.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace loadData {
namespace detail {

/**
 * Conversion from string to type. To be implemented for each type we want to read.
 */
template <typename T>
T fromString(const std::string& str);

template <>
inline std::string fromString<std::string>(const std::string& str) {
  return str;
}

template <>
inline std::size_t fromString<std::size_t>(const std::string& str) {
  return std::stoul(str);
}

template <>
inline int fromString<int>(const std::string& str) {
  return std::stoi(str);
}

template <>
inline double fromString<double>(const std::string& str) {
  return std::stod(str);
}

template <>
inline float fromString<float>(const std::string& str) {
  return std::stof(str);
}

/**
 * Extends a pair with methods to read from and to strings
 * @tparam T1
 * @tparam T2
 */
template <typename T1, typename T2>
struct ExtendedPair {
  T1 first;
  T2 second;

  using stdPair = std::pair<T1, T2>;

  // Constructor
  ExtendedPair(T1 firstIn, T2 secondIn) : first(std::move(firstIn)), second(std::move(secondIn)) {}

  // Constructor from string
  explicit ExtendedPair(const std::string& str) : first(), second() {
    using separator = boost::char_separator<char>;

    std::vector<std::string> container;
    container.reserve(2);

    boost::tokenizer<separator> tokens(str, separator(", "));
    std::copy(tokens.begin(), tokens.end(), std::back_inserter(container));
    if (container.size() != 2) {
      const std::string msg = "Failed parsing pair: \"" + str + R"(". Expected: x,x (no spaces) or "x, x")";
      throw std::runtime_error(msg);
    } else {
      first = fromString<T1>(container[0]);
      second = fromString<T2>(container[1]);
    }
  }

  // conversion operator to std::pair
  explicit operator stdPair() const { return std::pair<T1, T2>(first, second); }

  // stream operator
  friend std::ostream& operator<<(std::ostream& stream, const ExtendedPair& pair) {
    stream << "[" << pair.first << ", " << pair.second << "]";
    return stream;
  }

  // conversion to string, relies on the type being compatible with the stream operator.
  std::string toString() const {
    std::ostringstream stream;
    stream << *this;
    return stream.str();
  }
};

template <typename T1, typename T2>
struct translator_between_impl {
  using internal_type = std::string;
  using external_type = ExtendedPair<T1, T2>;
  boost::optional<external_type> get_value(const internal_type& str) { return boost::optional<external_type>(external_type(str)); }
  boost::optional<internal_type> put_value(const external_type& obj) { return boost::optional<internal_type>(obj.toString()); }
};

}  // namespace detail
}  // namespace loadData
}  // namespace ocs2

namespace boost {
namespace property_tree {

template <typename T1, typename T2>
struct translator_between<std::string, ocs2::loadData::detail::ExtendedPair<T1, T2>> {
  using type = ocs2::loadData::detail::translator_between_impl<T1, T2>;
};

}  // namespace property_tree
}  // namespace boost

namespace ocs2 {
namespace loadData {

/**
 * An auxiliary function which loads a vector of string pairs from a file.
 *
 * It has the following format:	<br>
 * topicName	<br>
 * {	<br>
 *   [0] "value1, value2"	<br>
 *   [2] "value1, value2"	<br>
 *   [1] "value1, value2"	<br>
 * } 	<br>
 *
 * @tparam T1 : first type of the pair
 * @tparam T2 : second type of the pair
 * @param [in] filename : File name which contains the configuration data.
 * @param [in] topicName : The key name assigned in the config file.
 * @param [out] loadVector : The loaded vector of pairs of strings.
 * @param [in] verbose : Print values as they are loaded
 */
template <typename T1, typename T2>
void loadStdVectorOfPair(const std::string& filename, const std::string& topicName, std::vector<std::pair<T1, T2>>& loadVector,
                         bool verbose = true) {
  std::vector<detail::ExtendedPair<T1, T2>> extendedPairVector;
  loadStdVector(filename, topicName, extendedPairVector, verbose);
  if (!extendedPairVector.empty()) {
    loadVector.clear();
    auto toStdPair = [](const detail::ExtendedPair<T1, T2>& p) { return std::pair<T1, T2>(p); };
    std::transform(extendedPairVector.begin(), extendedPairVector.end(), std::back_inserter(loadVector), toStdPair);
  }
}

}  // namespace loadData
}  // namespace ocs2
