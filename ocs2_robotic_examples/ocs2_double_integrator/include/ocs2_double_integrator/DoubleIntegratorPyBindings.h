#pragma once

#include <ocs2_double_integrator/DoubleIntegratorInterface.h>
#include <ocs2_double_integrator/definitions.h>
#include <ocs2_python_interface/PythonInterface.h>

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorPyBindings final : public PythonInterface {
 public:
  explicit DoubleIntegratorPyBindings(const std::string& taskFileFolder) {
    DoubleIntegratorInterface doubleIntegratorInterface(taskFileFolder);
    PythonInterface::init(doubleIntegratorInterface, doubleIntegratorInterface.getMpc());
  }
};

}  // namespace double_integrator
}  // namespace ocs2
