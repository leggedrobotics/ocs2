#include <ocs2_oc/pi_solver/PiSolver.hpp>

#include <ocs2_double_slit_example/DoubleSlitInterface.h>
#include <ocs2_double_slit_example/ros_comm/MPC_ROS_Double_Slit.h>
#include <ocs2_mpc/MPC_PI.h>

#include <cfenv>

int main(int argc, char* argv[]) {
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }

  // instantiate interface
  ocs2::double_slit::DoubleSlitInterface doubleSlitInterface(argv[1]);

  // MPC ROS Node
  ocs2::double_slit::MpcRosDoubleSlit mpcNode(*(doubleSlitInterface.getPiMpcPtr()), "double_slit");
  mpcNode.launchNodes(argc, argv);

  // Successful exit
  return 0;
}
