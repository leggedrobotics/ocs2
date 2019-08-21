#include <ocs2_quadrotor_example/QuadrotorInterface.h>
#include <ocs2_quadrotor_example/QuadrotorPyBindings.hpp>

namespace ocs2 {
namespace quadrotor {

QuadrotorPyBindings::QuadrotorPyBindings(const std::string& taskFileFolder, bool async) : Base(async) { init(taskFileFolder); }

void QuadrotorPyBindings::initRobotInterface(const std::string& taskFileFolder) {
  robotInterface_.reset(new QuadrotorInterface(taskFileFolder));
}

}  // namespace quadrotor
}  // namespace ocs2
