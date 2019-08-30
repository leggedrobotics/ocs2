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

  explicit NetworkController(const std::string& networkFilePath) { loadNetwork(networkFilePath); }

  void loadNetwork(const std::string& filePath) { policyNet_ = torch::jit::load(filePath); }

  static std::pair<double, double> timeTransform(double t) {
    if (t > 2.8) {
      return {0.0, 0.0};
    } else {
      return {std::max(0.0, sin(2.0 * M_PI * t / 0.7)), std::max(0.0, -sin(2.0 * M_PI * t / 0.7))};
    }
  }

  static std::array<double, 4> timeTransformStaticWalk(double t) {
    std::array<double, 4> res = {0.0, 0.0, 0.0, 0.0};
    if (t < 2.8) {
      t = std::fmod(t, 1.4);
      res[static_cast<int>(t / 0.35)] = std::abs(sin(2.0 * M_PI * t / 0.7));
    }
    return res;
  }

  input_vector_t computeInput(const scalar_t& t, const state_vector_t& x) override {
    //    Eigen::Matrix<float, STATE_DIM + 2, 1> ttx_float;
    Eigen::Matrix<float, STATE_DIM + 4, 1> ttx_float;
    // auto tt = timeTransform(t);
    auto tt = timeTransformStaticWalk(t);
    //    ttx_float << std::get<0>(tt), std::get<1>(tt), x.template cast<float>();
    ttx_float << tt[0], tt[1], tt[2], tt[3], x.template cast<float>();
    std::cout << "timeTransform " << tt[0] << " | " << tt[1] << " | " << tt[2] << " | " << tt[3] << std::endl;
    auto torch_tx = torch::from_blob(ttx_float.data(), ttx_float.size());

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch_tx);

    auto outputs = policyNet_.forward(inputs).toTuple();
    torch::Tensor p = outputs->elements()[0].toTensor();
    torch::Tensor u_1 = outputs->elements()[1].toTensor();

    Eigen::Matrix<float, INPUT_DIM, 1> u_float = Eigen::Map<Eigen::Matrix<float, INPUT_DIM, 1>>(u_1.data<float>(), INPUT_DIM);

    return u_float.template cast<scalar_t>();
  }

  void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override {
    throw std::runtime_error("not implemented.");
  }

  void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override {
    throw std::runtime_error("not implemented.");
  }

  void concatenate(const ControllerBase<STATE_DIM, INPUT_DIM>* nextController) override { throw std::runtime_error("not implemented."); }

  ControllerType getType() const override { return ControllerType::NETWORK; }

  void clear() override {}

  void setZero() override { throw std::runtime_error("not implemented."); }

  bool empty() const override { return false; }

 protected:
  torch::jit::script::Module policyNet_;
};
}  // namespace ocs2
