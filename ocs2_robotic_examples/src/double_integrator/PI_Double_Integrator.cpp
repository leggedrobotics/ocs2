#include <ocs2_oc/pi_solver/PiSolver.hpp>

#include <ocs2_mpc/MPC_PI.h>
#include <ocs2_robotic_examples/examples/double_integrator/DoubleIntegratorInterface.h>
#include <ocs2_robotic_examples/examples/double_integrator/ros_comm/MPC_ROS_Double_Integrator.h>

#include <ocs2_core/misc/loadEigenMatrix.h>

int main(int argc, char** argv) {
  // task file
  if (argc <= 1) throw std::runtime_error("No task file specified. Aborting.");

  std::string taskFile = std::string(PACKAGE_PATH) + "/config/double_integrator/" + std::string(argv[1]) + "/task.info";
  std::cerr << "Loading task file: " << taskFile << std::endl;

  using dynamics_t = ocs2::double_integrator::DoubleIntegratorDynamics;
  constexpr auto STATE_DIM = dynamics_t::DIMENSIONS::STATE_DIM_;
  constexpr auto INPUT_DIM = dynamics_t::DIMENSIONS::INPUT_DIM_;

  // Dynamics
  dynamics_t::scalar_t mass;
  ocs2::loadScalar(taskFile, "systemParameters.mass", mass);
  dynamics_t::Ptr dynamics(new dynamics_t(mass));

  // Initial state
  dynamics_t::DIMENSIONS::state_vector_t xInitial;
  ocs2::loadEigenMatrix(taskFile, "initialState", xInitial);

  // Cost function
  using cost_t = ocs2::double_integrator::DoubleIntegratorCost;
  dynamics_t::DIMENSIONS::state_matrix_t Q, QFinal;
  ocs2::loadEigenMatrix(taskFile, "Q", Q);
  ocs2::loadEigenMatrix(taskFile, "Q_final", QFinal);

  dynamics_t::DIMENSIONS::input_matrix_t R;
  ocs2::loadEigenMatrix(taskFile, "R", R);

  dynamics_t::DIMENSIONS::state_vector_t xFinal;
  ocs2::loadEigenMatrix(taskFile, "x_final", xFinal);

  std::cerr << "Q:  \n" << Q << std::endl;
  std::cerr << "R:  \n" << R << std::endl;
  std::cerr << "Q_final:\n" << QFinal << std::endl;
  std::cerr << "x_init:   " << xInitial.transpose() << std::endl;
  std::cerr << "x_final:  " << xFinal.transpose() << std::endl;
  cost_t::Ptr cost(new cost_t(Q, R, QFinal));

  dynamics_t::scalar_t initTime(0.0);
  dynamics_t::scalar_t finalTime(5.0);
  using cost_desired_trajectories_t = ocs2::PiSolver<STATE_DIM, INPUT_DIM>::cost_desired_trajectories_t;
  cost_desired_trajectories_t::scalar_array_t desiredTimeArray{initTime, finalTime};
  cost_desired_trajectories_t::dynamic_vector_array_t desiredStateArray(2), desiredInputArray(2);
  desiredStateArray[0] = xFinal;
  desiredStateArray[1] = xFinal;
  desiredInputArray[0].setZero(INPUT_DIM);
  desiredInputArray[1].setZero(INPUT_DIM);

  cost_desired_trajectories_t costDesiredTraj(desiredTimeArray, desiredStateArray, desiredInputArray);

  // MPC ROS Node
  ocs2::double_integrator::DoubleIntegratorInterface doubleIntegratorInterface(argv[1]);
  ocs2::MPC_PI<STATE_DIM, INPUT_DIM> mpc_pi(dynamics, cost, doubleIntegratorInterface.mpcSettings());
  //  mpc_pi.setCostDesiredTrajectories(costDesiredTraj);
  ocs2::double_integrator::MPC_ROS_Linear_System mpcNode(mpc_pi, "double_integrator");

  mpcNode.launchNodes(argc, argv);

  // Successful exit
  return 0;
}
