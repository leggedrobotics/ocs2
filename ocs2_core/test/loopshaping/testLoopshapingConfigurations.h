/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <gtest/gtest.h>
#include <experimental/filesystem>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/LoopshapingPreComputation.h>
#include <ocs2_core/loopshaping/LoopshapingPropertyTree.h>

namespace ocs2 {

const std::vector<std::string> configNames = {"loopshaping_r.conf", "loopshaping_r_ballbot.conf", "loopshaping_s.conf",
                                              "loopshaping_s_integrator.conf"};

inline std::string getAbsolutePathToConfigurationFile(const std::string& fileName) {
  const std::experimental::filesystem::path pathToTest = std::experimental::filesystem::path(__FILE__);
  return std::string(pathToTest.parent_path()) + "/" + fileName;
}

class LoopshapingTestConfiguration {
 public:
  LoopshapingTestConfiguration(const std::string& configName);

 protected:
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
  std::unique_ptr<PreComputation> preComp_sys_;
  std::unique_ptr<LoopshapingPreComputation> preComp_;

  size_t systemStateDim_;
  size_t filterStateDim_;
  size_t inputDim_;

  const scalar_t tol = 1e-9;

  scalar_t t;
  vector_t x_;
  vector_t u_;
  vector_t x_sys_;
  vector_t u_sys_;
  vector_t x_filter_;
  vector_t u_filter_;

  vector_t x_disturbance_;
  vector_t u_disturbance_;
  vector_t x_sys_disturbance_;
  vector_t u_sys_disturbance_;
  vector_t x_filter_disturbance_;
  vector_t u_filter_disturbance_;

 private:
  void getRandomStateInput(size_t systemStateDim, size_t filterStateDim, size_t inputDim, vector_t& x_sys, vector_t& u_sys,
                           vector_t& x_filter, vector_t& u_filter, vector_t& x, vector_t& u, scalar_t range = 1.0);
};

}  // namespace ocs2
