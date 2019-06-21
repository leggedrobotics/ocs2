#include <ocs2_double_integrator_noros_example/DoubleIntegratorPyBindings.hpp>
#include <ocs2_double_integrator_noros_example/DoubleIntegratorInterface.h>

namespace ocs2 {
namespace double_integrator {

void DoubleIntegratorPyBindings::initRobotInterface(const std::string& taskFileFolder) {

  robotInterface_.reset(new ocs2::double_integrator::DoubleIntegratorInterface(taskFileFolder));

}

}  // namespace double_integrator
}  // namespace ocs2
