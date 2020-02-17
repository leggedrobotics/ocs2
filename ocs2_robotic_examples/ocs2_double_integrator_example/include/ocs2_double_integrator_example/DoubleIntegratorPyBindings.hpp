#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include <ocs2_double_integrator_example/definitions.h>

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorPyBindings final : public PythonInterface<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> {
 public:
  using Base = PythonInterface<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_>;

  explicit DoubleIntegratorPyBindings(const std::string& taskFileFolder, bool async = false);
};

}  // namespace double_integrator
}  // namespace ocs2
