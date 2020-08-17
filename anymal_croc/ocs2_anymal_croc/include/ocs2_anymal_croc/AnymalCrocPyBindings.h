#pragma once

#include <ocs2_python_interface/PythonInterface.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

namespace anymal {

class AnymalCrocPyBindings final : public ocs2::PythonInterface {
 public:
  explicit AnymalCrocPyBindings(std::string taskName);

  void visualizeTrajectory(const ocs2::scalar_array_t& t, const ocs2::vector_array_t& x, const ocs2::vector_array_t& u,
                           ocs2::scalar_t speed) override;

 private:
  std::unique_ptr<switched_model::QuadrupedVisualizer> visualizer_;
  std::string taskName_;
};

}  // namespace anymal
