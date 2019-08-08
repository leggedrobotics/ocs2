#include <ocs2_ballbot_example/BallbotInterface.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_mpc/MPC_PI.h>

//#include <cfenv>

int main(int argc, char* argv[]) {
//  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }

  ocs2::ballbot::BallbotInterface ballbotInterface(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  ocs2::MPC_ROS_Interface<ocs2::ballbot::STATE_DIM_, ocs2::ballbot::INPUT_DIM_> mpcNode(ballbotInterface.getMpcPiPtr(), "ballbot");
  mpcNode.launchNodes(argc, argv);

  return 0;
}
