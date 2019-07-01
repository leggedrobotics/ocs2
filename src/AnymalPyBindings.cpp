#include <ocs2_anymal_interface/AnymalPyBindings.h>

namespace anymal {

AnymalPyBindings::AnymalPyBindings(const std::string& taskFileFolder, bool async) : Base(async) { init(taskFileFolder); }

void AnymalPyBindings::initRobotInterface(const std::string& taskFileFolder) {
  robotInterface_.reset(new OCS2AnymalInterface(taskFileFolder));
}

}  // namespace anymal
