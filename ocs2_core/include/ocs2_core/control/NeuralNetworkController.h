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

#include <functional>
#include <string>
#include <vector>

#include <ocs2_core/control/ControllerBase.h>
#include <torch/script.h>

namespace ocs2 {

class NeuralNetworkController final : public ControllerBase {
 public:
  using state_in_transform_fct_t = std::function<Eigen::VectorXf(scalar_t, const vector_t&)>;
  using control_out_transform_fct_t = std::function<vector_t(const torch::IValue&)>;

  NeuralNetworkController() = default;

  NeuralNetworkController(const std::string& networkFilePath, state_in_transform_fct_t state_in_transform_fct,
                          control_out_transform_fct_t control_out_transform_fct);

  void loadNetwork(const std::string& filePath);

  input_vector_t computeInput(scalar_t t, const vector_t& x) override;

  void concatenate(const ControllerBase* otherController, int index, int length) override;

  ControllerType getType() const override;

  void clear() override;

  void setZero() override;

  bool empty() const override;

 protected:
  torch::jit::script::Module policyNet_;

  // transformation functions
  state_in_transform_fct_t state_in_transform_fct_;
  control_out_transform_fct_t control_out_transform_fct_;
};

}  // namespace ocs2
