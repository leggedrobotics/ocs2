#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

namespace anymal {

class AnymalBearPyBindings final : public ocs2::PythonInterface<switched_model::STATE_DIM, switched_model::INPUT_DIM> {
 public:
  using Base = ocs2::PythonInterface<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
  using visualizer_t = switched_model::QuadrupedVisualizer;

  explicit AnymalBearPyBindings(std::string taskName, bool async = false);

  void visualizeTrajectory(const scalar_array_t& t, const state_vector_array_t& x, const input_vector_array_t& u, double speed) override;

 private:
  std::unique_ptr<visualizer_t> visualizer_;
  std::string taskName_;
};

}  // namespace anymal
