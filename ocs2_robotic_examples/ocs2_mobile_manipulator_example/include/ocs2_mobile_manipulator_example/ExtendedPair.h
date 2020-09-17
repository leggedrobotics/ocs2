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

#include <cstddef>

#include <cppad/cg.hpp>

#include <ocs2_core/Types.h>

#include <ocs2_core/misc/LoadData.h>
#include <boost/tokenizer.hpp>

namespace ocs2 {

template <typename FirstType, typename SecondType>
class ExtendedPair {
public:
  FirstType first;
  SecondType second;

  friend std::ostream& operator<<(std::ostream& stream, const ExtendedPair& pair){
    stream << "["<<pair.first<<", "<<pair.second<<"]";
    return stream;
  }
};

}  // namespace mobile_manipulator


namespace boost {
namespace property_tree {

template<>
struct translator_between<std::string, ocs2::ExtendedPair<size_t, size_t>>
{
  struct type {
    typedef std::string internal_type;
    typedef ocs2::ExtendedPair<size_t, size_t> external_type;
    boost::optional<external_type> get_value(const internal_type& str){
      std::vector<std::string> container;
      typedef boost::char_separator<char> separator;
      std::cout<<"str = "<<str<<std::endl;
      boost::tokenizer<separator> tokens(str, separator(", "));
      std::copy(tokens.begin(), tokens.end(), std::back_inserter(container)); 
      external_type pair;
      if (container.size() == 2){
        pair.first = std::stoi(container[0]);
        pair.second = std::stoi(container[1]);
      } else if (container.size() == 1){
        std::stringstream msg;
        msg<<"Expected size 2 for a pair, however got only one value: "<<container[0]<<". The pair should be written as x,x (no spaces) or \"x, x\"";
        throw std::runtime_error(msg.str());
      } else if (container.size() == 0){
        std::stringstream msg;
        msg<<"Expected size 2 for a pair, however got no values?"<<std::endl;
        throw std::runtime_error(msg.str());
      } else{
        std::stringstream msg;
        msg<<"Expected size 2 for a pair, however got too many values? String was "<<str<<", tokens were "<<std::endl;
        for (const auto& element : container){
          msg << element <<", ";
        }
        throw std::runtime_error(msg.str());
      }
      return boost::optional<external_type>(pair);
    }
    boost::optional<internal_type> put_value(const external_type& obj){
      std::string returnval =std::to_string(obj.first) + ", " + std::to_string(obj.second);
      return boost::optional<internal_type>(returnval);
    }
  };
};

} // namespace property_tree
} // namespace boost

