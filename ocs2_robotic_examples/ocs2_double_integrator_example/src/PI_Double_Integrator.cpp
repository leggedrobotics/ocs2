#include <ocs2_oc/pi_solver/PiSolver.hpp>

#include <ocs2_core/cost/PathIntegralCostFunction.h>
#include <ocs2_mpc/MPC_PI.h>
#include <ocs2_double_integrator_example/dynamics/DoubleIntegratorDynamics.h>
//#include <ocs2_double_integrator_example/dynamics/DoubleIntegratorInterface.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_double_integrator_example/ros_comm/MPC_ROS_Double_Integrator.h>
#include <ros/package.h>

#include <ocs2_core/misc/loadEigenMatrix.h>

int main(int argc, char** argv) {
  // task file
  if (argc <= 1) throw std::runtime_error("No task file specified. Aborting.");

  std::string taskFile =
      ros::package::getPath("ocs2_robotic_examples") + "/config/double_integrator/" + std::string(argv[1]) + "/task.info";

  using dynamics_t = ocs2::double_integrator::DoubleIntegratorDynamics;
  constexpr auto STATE_DIM = dynamics_t::DIMENSIONS::STATE_DIM_;
  constexpr auto INPUT_DIM = dynamics_t::DIMENSIONS::INPUT_DIM_;
  using solver_t = ocs2::PiSolver<STATE_DIM, INPUT_DIM>;

  // Dynamics
  dynamics_t::scalar_t mass;
  ocs2::loadScalar(taskFile, "systemParameters.mass", mass);
  dynamics_t::Ptr dynamics(new dynamics_t(mass));

  // Initial state
  dynamics_t::DIMENSIONS::state_vector_t xInitial;
  ocs2::loadEigenMatrix(taskFile, "initialState", xInitial);

  // Cost function
  dynamics_t::DIMENSIONS::state_matrix_t Q, QFinal;
  ocs2::loadEigenMatrix(taskFile, "Q", Q);
  ocs2::loadEigenMatrix(taskFile, "Q_final", QFinal);

  dynamics_t::DIMENSIONS::input_matrix_t R;
  ocs2::loadEigenMatrix(taskFile, "R", R);

  dynamics_t::DIMENSIONS::state_vector_t xFinal;
  ocs2::loadEigenMatrix(taskFile, "x_final", xFinal);

  dynamics_t::DIMENSIONS::input_vector_t uNominal;
  uNominal.setZero();

  auto V = [Q](const dynamics_t::DIMENSIONS::state_vector_t& x) { return x.dot(Q * x); };
  auto r = [](const dynamics_t::DIMENSIONS::state_vector_t& x) { return dynamics_t::DIMENSIONS::input_vector_t::Zero(); };
  auto Phi = [QFinal](const dynamics_t::DIMENSIONS::state_vector_t& x) { return x.dot(QFinal * x); };

  std::cerr << "Q:  \n" << Q << std::endl;
  std::cerr << "R:  \n" << R << std::endl;
  std::cerr << "Q_final:\n" << QFinal << std::endl;
  std::cerr << "x_init:   " << xInitial.transpose() << std::endl;
  std::cerr << "x_final:  " << xFinal.transpose() << std::endl;
  using cost_t = ocs2::PathIntegralCostFunction<STATE_DIM, INPUT_DIM>;
  std::unique_ptr<cost_t> cost(new cost_t(R, uNominal, V, r, Phi));

  dynamics_t::scalar_t initTime(0.0);
  dynamics_t::scalar_t finalTime(5.0);
  using cost_desired_trajectories_t = solver_t::cost_desired_trajectories_t;
  cost_desired_trajectories_t::scalar_array_t desiredTimeArray{initTime, finalTime};
  cost_desired_trajectories_t::dynamic_vector_array_t desiredStateArray(2), desiredInputArray(2);
  desiredStateArray[0] = xFinal;
  desiredStateArray[1] = xFinal;
  desiredInputArray[0].setZero(INPUT_DIM);
  desiredInputArray[1].setZero(INPUT_DIM);
  cost_desired_trajectories_t costDesiredTraj(desiredTimeArray, desiredStateArray, desiredInputArray);

  // constraint
  solver_t::constraint_t constraint;

  constexpr double rollout_dt = 0.01;

  // MPC ROS Node
  // ocs2::double_integrator::DoubleIntegratorInterface doubleIntegratorInterface(argv[1]);
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(taskFile);
  ocs2::MPC_PI<STATE_DIM, INPUT_DIM> mpc_pi(dynamics, std::move(cost), constraint, rollout_dt, mpcSettings);
  mpc_pi.setCostDesiredTrajectories(costDesiredTraj);
  ocs2::double_integrator::MPC_ROS_Linear_System mpcNode(mpc_pi, "double_integrator");

  mpcNode.launchNodes(argc, argv);

  // Successful exit
  return 0;
}
