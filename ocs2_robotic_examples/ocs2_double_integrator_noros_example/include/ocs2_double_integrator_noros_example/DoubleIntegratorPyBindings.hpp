#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include <ocs2_double_integrator_noros_example/definitions.h>

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorPyBindings final : public PythonInterface<double_integrator_dims::STATE_DIM_,
    double_integrator_dims::INPUT_DIM_>
{
 public:
  using Base = PythonInterface<ocs2::double_integrator::double_integrator_dims::STATE_DIM_,
                               ocs2::double_integrator::double_integrator_dims::INPUT_DIM_>;

  DoubleIntegratorPyBindings(const std::string& taskFileFolder, bool async=false);

  void initRobotInterface(const std::string& taskFileFolder) override;
};

}  // namespace double_integrator
}  // namespace ocs2
