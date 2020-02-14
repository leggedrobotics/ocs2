#include <ocs2_ballbot_example/BallbotInterface.h>
#include <ocs2_ballbot_example/BallbotPyBindings.h>

namespace ocs2 {
namespace ballbot {

BallbotPyBindings::BallbotPyBindings(const std::string& taskFileFolder, bool async) : Base(async) {
  BallbotInterface ballbotInterface(taskFileFolder);
  init(ballbotInterface, ballbotInterface.getMpc());
}

}  // namespace ballbot
}  // namespace ocs2
