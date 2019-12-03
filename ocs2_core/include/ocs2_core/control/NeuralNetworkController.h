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

#pragma once

#include <ocs2_core/control/ControllerBase.h>
#include <torch/script.h>

namespace ocs2 {
template <size_t STATE_DIM, size_t INPUT_DIM>
class NeuralNetworkController final : public ControllerBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ControllerBase<STATE_DIM, INPUT_DIM>;

  using dimensions_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimensions_t::scalar_t;
  using scalar_array_t = typename dimensions_t::scalar_array_t;
  using float_array_t = typename Base::float_array_t;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using state_vector_array_t = typename dimensions_t::state_vector_array_t;
  using input_vector_t = typename dimensions_t::input_vector_t;
  using input_vector_array_t = typename dimensions_t::input_vector_array_t;
  using input_state_matrix_t = typename dimensions_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename dimensions_t::input_state_matrix_array_t;

  using state_in_transform_fct_t = std::function<Eigen::VectorXf(scalar_t, const state_vector_t&)>;
  using control_out_transform_fct_t = std::function<input_vector_t(const torch::IValue&)>;

  NeuralNetworkController() = default;

  explicit NeuralNetworkController(const std::string& networkFilePath, state_in_transform_fct_t state_in_transform_fct,
                                   control_out_transform_fct_t control_out_transform_fct)
      : state_in_transform_fct_(std::move(state_in_transform_fct)), control_out_transform_fct_(std::move(control_out_transform_fct)) {
    loadNetwork(networkFilePath);
  }

  void loadNetwork(const std::string& filePath) { policyNet_ = torch::jit::load(filePath); }

  input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    Eigen::VectorXf net_input_float = state_in_transform_fct_(t, x);
    auto input_torch = torch::from_blob(net_input_float.data(), net_input_float.size());
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_torch);

    auto output = policyNet_.forward(inputs);
    return control_out_transform_fct_(output);
  }

  void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override {
    throw std::runtime_error("not implemented.");
  }

  void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override {
    throw std::runtime_error("not implemented.");
  }

  void concatenate(const ControllerBase<STATE_DIM, INPUT_DIM>* nextController) override { throw std::runtime_error("not implemented."); }

  ControllerType getType() const override { return ControllerType::NEURAL_NETWORK; }

  void clear() override {}

  void setZero() override { throw std::runtime_error("not implemented."); }

  bool empty() const override { return false; }

 protected:
  torch::jit::script::Module policyNet_;

  // transformation functions
  state_in_transform_fct_t state_in_transform_fct_;
  control_out_transform_fct_t control_out_transform_fct_;
};
}  // namespace ocs2
