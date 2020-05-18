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

#include <ocs2_oc/rollout/RootFinder.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RootFinder::setInitBracket(pair_t timeInt, pair_t guardInt) {
  if (guardInt.first * guardInt.second > 0) {
    throw std::runtime_error("Bracket function values should have opposite signs!");
  }
  timeInt_ = timeInt;
  guardInt_ = guardInt;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RootFinder::updateBracket(const scalar_t& query, const scalar_t& fQuery) {
  if (fQuery * guardInt_.first < 0) {
    guardInt_.second = guardInt_.first;
    timeInt_.second = timeInt_.first;

    guardInt_.first = fQuery;
    timeInt_.first = query;

  } else {
    scalar_t gamma;
    switch (rootFindingAlgorithm_) {
      case (RootFinderType::ANDERSON_BJORCK): {
        gamma = 1 - (fQuery / guardInt_.first);
        if (gamma < 0) {
          gamma = 0.5;
        }
        break;
      }
      case (RootFinderType::PEGASUS): {
        gamma = guardInt_.first / (guardInt_.first + fQuery);
        break;
      }
      case (RootFinderType::ILLINOIS): {
        gamma = 0.5;
        break;
      }
      case (RootFinderType::REGULA_FALSI): {
        gamma = 1.0;
        break;
      }
      default: {
        throw std::runtime_error("Root finding algorithm of type " +
                                 std::to_string(static_cast<std::underlying_type<RootFinderType>::type>(rootFindingAlgorithm_)) +
                                 " is not supported.");
      }
    }

    guardInt_.first = fQuery;
    timeInt_.first = query;

    guardInt_.second *= gamma;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RootFinder::display() {
  std::cerr << "Root Finding Information\n";
  std::cerr << "Time Bracket:  [" << timeInt_.first << ";" << timeInt_.second << "]\n";
  std::cerr << "Value Bracket: [" << guardInt_.first << ";" << guardInt_.second << "]\n";
}
}  // namespace ocs2
