#pragma once

#include <ocs2_core/control/ControllerBase.h>

#include <torch/script.h>

namespace ocs2 {
template <size_t STATE_DIM, size_t INPUT_DIM>
class NetworkController final : public ControllerBase<STATE_DIM, INPUT_DIM> {
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

  NetworkController() = default;

  void loadNetwork(std::string filePath) { policyNet_ = torch::jit::load(filePath);}

  input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::ones({1, 3, 224, 224}));

    // Execute the model and turn its output into a tensor.
    at::Tensor output = policyNet_.forward(inputs).toTensor();

    input_vector_t u = Eigen::Map<input_vector_t>(output.data<scalar_t>(), INPUT_DIM);
    return u;
  }

  void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override {
    throw std::runtime_error("not implemented.");
  }

  void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override {
    throw std::runtime_error("not implemented.");
  }

  ControllerType getType() const override { return ControllerType::NETWORK; }

  void clear() override { }

  void setZero() override { throw std::runtime_error("not implemented."); }

  bool empty() const override { return false; }

 protected:
  torch::jit::script::Module policyNet_;
};
}  // namespace ocs2
