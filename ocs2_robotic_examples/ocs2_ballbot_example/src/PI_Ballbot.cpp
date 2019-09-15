#include <ocs2_ballbot_example/BallbotInterface.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_mpc/MPC_PI.h>

using namespace ocs2;

int main(int argc, char* argv[]) {
  //  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }

  ballbot::BallbotInterface ballbotInterface(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  MPC_ROS_Interface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_> mpcNode(ballbotInterface.getMpcPi(), "ballbot");
  mpcNode.launchNodes(argc, argv);

  return 0;
}
