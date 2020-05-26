#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include "definitions.h"

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorPyBindings final : public PythonInterface {
 public:
  explicit DoubleIntegratorPyBindings(const std::string& taskFileFolder, bool async = false);
};

}  // namespace double_integrator
}  // namespace ocs2
