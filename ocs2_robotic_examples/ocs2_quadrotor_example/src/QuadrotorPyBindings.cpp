#include <ocs2_quadrotor_example/QuadrotorInterface.h>
#include <ocs2_quadrotor_example/QuadrotorPyBindings.hpp>

namespace ocs2 {
namespace quadrotor {

QuadrotorPyBindings::QuadrotorPyBindings(const std::string& taskFileFolder, bool async) : Base(async) {
  QuadrotorInterface quadrotorInterface(taskFileFolder);
  init(quadrotorInterface, quadrotorInterface.getMpc());
}

}  // namespace quadrotor
}  // namespace ocs2
