#pragma once

#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>

namespace ocs2 {
namespace ballbot {

class BallbotPyBindings final : public PythonInterface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_> {
 public:
  using Base = PythonInterface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;

  BallbotPyBindings(const std::string& taskFileFolder, bool async = false);

  void initRobotInterface(const std::string& taskFileFolder) override;
};

}  // namespace ballbot
}  // namespace ocs2
