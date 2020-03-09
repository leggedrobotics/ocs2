//
// Created by rgrandia on 09.03.20.
//

#pragma once

#include <sstream>
#include <string>

namespace ocs2 {

/** Loops over the elements in a container and creates a delimited string */
template <typename Container>
std::string toDelimitedString(const Container& container, const std::string& delimiter = ", ") {
  std::stringstream ss;

  bool addDelimiter = false;
  for (const auto& value : container) {
    if (addDelimiter) {
      ss << delimiter;
    }
    ss << value;
    addDelimiter = true;
  }

  return ss.str();
}

}  // namespace ocs2
