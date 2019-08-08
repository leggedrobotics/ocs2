/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#ifndef MODESEQUENCE_OCS2_H_
#define MODESEQUENCE_OCS2_H_

#include <iostream>
#include <vector>

namespace ocs2 {

/**
 * Mode sequence template.
 *
 */
template <typename scalar_t = double>
struct ModeSequenceTemplate {
  ModeSequenceTemplate() : templateSwitchingTimes_(0), templateSubsystemsSequence_(0) {}

  /**
   * Defined as [t_0=0, t_1, .., t_n, t_(n+1)=T], where T is the overall duration
   * of the template logic. t_1 to t_n are the event moments.
   */
  std::vector<scalar_t> templateSwitchingTimes_;

  /**
   * Defined as [sys_0, sys_n], are the switching systems IDs. Here sys_i is
   * active in period [t_i, t_(i+1)]
   */
  std::vector<size_t> templateSubsystemsSequence_;

  /**
   * Displays template information.
   */
  void display() const {
    std::cerr << std::endl << "Template switching times:\n\t {";
    for (auto& s : templateSwitchingTimes_) {
      std::cerr << s << ", ";
    }
    if (!templateSwitchingTimes_.empty()) {
      std::cerr << "\b\b";
    }
    std::cerr << "}" << std::endl;

    std::cerr << "Template subsystem sequence:\n\t {";
    for (auto& s : templateSubsystemsSequence_) {
      std::cerr << s << ", ";
    }
    if (!templateSubsystemsSequence_.empty()) {
      std::cerr << "\b\b";
    }
    std::cerr << "}" << std::endl;
  }
};

}  // namespace ocs2

#endif /* MODESEQUENCE_OCS2_H_ */
