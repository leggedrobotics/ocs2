#pragma once

#include <ocs2_python_interface/PythonInterface.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

namespace anymal {

class AnymalPyBindings final : public ocs2::PythonInterface {
 public:
  explicit AnymalPyBindings(const std::string& taskFile, const std::string& libraryFolder, const std::string urdfFile = "");

  void visualizeTrajectory(const ocs2::scalar_array_t& t, const ocs2::vector_array_t& x, const ocs2::vector_array_t& u,
                           ocs2::scalar_t speed) override;

 private:
  std::unique_ptr<switched_model::QuadrupedVisualizer> visualizer_;
  std::string robotName_;
  std::string configName_;
};

}  // namespace anymal
