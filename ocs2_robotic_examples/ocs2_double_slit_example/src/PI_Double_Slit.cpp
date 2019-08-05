#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_double_slit_example/DoubleSlitInterface.h>
#include <ocs2_double_slit_example/definitions.h>
#include <ocs2_mpc/MPC_PI.h>
#include <ocs2_oc/pi_solver/PiSolver.hpp>

#include <cfenv>

int main(int argc, char* argv[]) {
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }

  // instantiate interface
  ocs2::double_slit::DoubleSlitInterface doubleSlitInterface(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // MPC ROS Node
  ocs2::MPC_ROS_Interface<ocs2::double_slit::STATE_DIM_, ocs2::double_slit::INPUT_DIM_> mpcNode(doubleSlitInterface.getPiMpcPtr(),
                                                                                                "double_slit");
  mpcNode.launchNodes(argc, argv);

  // Successful exit
  return 0;
}
