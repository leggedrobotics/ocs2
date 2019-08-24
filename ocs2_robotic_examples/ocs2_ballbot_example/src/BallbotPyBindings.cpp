#include <ocs2_ballbot_example/BallbotInterface.h>
#include <ocs2_ballbot_example/BallbotPyBindings.h>

namespace ocs2 {
namespace ballbot {

BallbotPyBindings::BallbotPyBindings(const std::string& taskFileFolder, bool async) : Base(async) {
  init(taskFileFolder);
}

void BallbotPyBindings::initRobotInterface(const std::string& taskFileFolder) {
  robotInterface_.reset(new BallbotInterface(taskFileFolder));
}

}  // namespace ballbot
}  // namespace ocs2
