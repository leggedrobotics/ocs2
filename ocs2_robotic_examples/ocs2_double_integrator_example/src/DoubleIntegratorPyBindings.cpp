#include <ocs2_double_integrator_example/DoubleIntegratorInterface.h>
#include <ocs2_double_integrator_example/DoubleIntegratorPyBindings.hpp>

namespace ocs2 {
namespace double_integrator {

DoubleIntegratorPyBindings::DoubleIntegratorPyBindings(const std::string& taskFileFolder, bool async) : Base(async) {
  init(taskFileFolder);
}

void DoubleIntegratorPyBindings::initRobotInterface(const std::string& taskFileFolder) {
  robotInterface_.reset(new DoubleIntegratorInterface(taskFileFolder));
}

}  // namespace double_integrator
}  // namespace ocs2
