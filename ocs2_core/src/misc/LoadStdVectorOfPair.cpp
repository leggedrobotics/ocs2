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

#include <algorithm>
#include <string>
#include <type_traits>
#include <utility>

#include <boost/tokenizer.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_core/misc/LoadStdVectorOfPair.h"

namespace ocs2 {
namespace {

template <typename T1, typename T2>
struct ExtendedPair {
  T1 first;
  T2 second;

  using stdPair = std::pair<T1, T2>;

  // conversion operator to std::pair
  explicit operator stdPair() const { return std::pair<T1, T2>(first, second); }

  friend std::ostream& operator<<(std::ostream& stream, const ExtendedPair& pair) {
    stream << "[" << pair.first << ", " << pair.second << "]";
    return stream;
  }
};

}  // namespace
}  // namespace ocs2

namespace boost {
namespace property_tree {

template <>
struct translator_between<std::string, ocs2::ExtendedPair<size_t, size_t>> {
  struct type {
    using internal_type = std::string;
    using external_type = ocs2::ExtendedPair<size_t, size_t>;
    using separator = boost::char_separator<char>;

    boost::optional<external_type> get_value(const internal_type& str) {
      std::vector<std::string> container;
      boost::tokenizer<separator> tokens(str, separator(", "));
      std::copy(tokens.begin(), tokens.end(), std::back_inserter(container));
      if (container.size() != 2) {
        const std::string msg = "Failed parsing pair: \"" + str + R"(". Expected: x,x (no spaces) or "x, x")";
        throw std::runtime_error(msg);
      } else {
        external_type pair;
        pair.first = std::stoul(container[0]);
        pair.second = std::stoul(container[1]);
        return boost::optional<external_type>(pair);
      }
    }

    boost::optional<internal_type> put_value(const external_type& obj) {
      std::string returnval = std::to_string(obj.first) + ", " + std::to_string(obj.second);
      return boost::optional<internal_type>(returnval);
    }
  };
};

template <>
struct translator_between<std::string, ocs2::ExtendedPair<std::string, std::string>> {
  struct type {
    using internal_type = std::string;
    using external_type = ocs2::ExtendedPair<std::string, std::string>;
    using separator = boost::char_separator<char>;

    boost::optional<external_type> get_value(const internal_type& str) {
      std::vector<std::string> container;
      boost::tokenizer<separator> tokens(str, separator(", "));
      std::copy(tokens.begin(), tokens.end(), std::back_inserter(container));
      if (container.size() != 2) {
        const std::string msg = "Failed parsing pair: \"" + str + R"(". Expected: x,x (no spaces) or "x, x")";
        throw std::runtime_error(msg);
      } else {
        external_type pair;
        pair.first = container[0];
        pair.second = container[1];
        return boost::optional<external_type>(pair);
      }
    }

    boost::optional<internal_type> put_value(const external_type& obj) {
      std::string returnval = obj.first + ", " + obj.second;
      return boost::optional<internal_type>(returnval);
    }
  };
};

}  // namespace property_tree
}  // namespace boost

namespace ocs2 {
namespace loadData {

template <typename T1, typename T2>
void loadStdVectorOfPairImpl(const std::string& filename, const std::string& topicName, std::vector<std::pair<T1, T2>>& loadVector,
                             bool verbose = true) {
  std::vector<ExtendedPair<T1, T2>> extendedPairVector;
  loadStdVector(filename, topicName, extendedPairVector, verbose);
  if (!extendedPairVector.empty()) {
    loadVector.clear();
    auto toStdPair = [](const ExtendedPair<T1, T2>& p) { return std::pair<T1, T2>(p); };
    std::transform(extendedPairVector.begin(), extendedPairVector.end(), std::back_inserter(loadVector), toStdPair);
  }
}

void loadStdVectorOfPair(const std::string& filename, const std::string& topicName,
                         std::vector<std::pair<std::string, std::string>>& loadVector, bool verbose) {
  loadStdVectorOfPairImpl(filename, topicName, loadVector, verbose);
}

void loadStdVectorOfPair(const std::string& filename, const std::string& topicName, std::vector<std::pair<size_t, size_t>>& loadVector,
                         bool verbose) {
  loadStdVectorOfPairImpl(filename, topicName, loadVector, verbose);
}

}  // namespace loadData
}  // namespace ocs2
