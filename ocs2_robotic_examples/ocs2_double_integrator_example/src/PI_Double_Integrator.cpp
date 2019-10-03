#include <ocs2_oc/pi_solver/PiSolver.hpp>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_double_integrator_example/cost/DoubleIntegratorCost.h>
#include <ocs2_double_integrator_example/dynamics/DoubleIntegratorDynamics.h>
#include <ocs2_mpc/MPC_PI.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/pi_solver/PI_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ros/package.h>

#include <ocs2_core/misc/LoadData.h>

int main(int argc, char** argv) {
  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }

  std::string taskFile = ros::package::getPath("ocs2_double_integrator_example") + "/config/" + std::string(argv[1]) + "/task.info";

  using DIMENSIONS = ocs2::MPC_PI<ocs2::double_integrator::STATE_DIM_, ocs2::double_integrator::INPUT_DIM_>::DIMENSIONS;
  constexpr auto STATE_DIM = DIMENSIONS::STATE_DIM_;
  constexpr auto INPUT_DIM = DIMENSIONS::INPUT_DIM_;
  using solver_t = ocs2::PiSolver<STATE_DIM, INPUT_DIM>;

  // Dynamics
  DIMENSIONS::state_matrix_t A;
  A << 0.0, 1.0, 0.0, 0.0;
  DIMENSIONS::state_input_matrix_t B;
  B << 0.0, 1.0;
  ocs2::double_integrator::DoubleIntegratorDynamics dynamics(A, B);

  // Initial state
  DIMENSIONS::state_vector_t xInitial;
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", xInitial);

  // Cost function
  DIMENSIONS::state_matrix_t Q, QFinal;
  ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
  ocs2::loadData::loadEigenMatrix(taskFile, "Q_final", QFinal);

  DIMENSIONS::input_matrix_t R;
  ocs2::loadData::loadEigenMatrix(taskFile, "R", R);

  DIMENSIONS::state_vector_t xFinal;
  ocs2::loadData::loadEigenMatrix(taskFile, "x_final", xFinal);

  DIMENSIONS::input_vector_t uNominal;
  uNominal.setZero();

  std::cerr << "Path Integral: Cost function parameters\n"
            << "Q:\n"
            << Q << "\nR:\n"
            << R << "\nQ_final:\n"
            << QFinal << "\nx_init:   " << xInitial.transpose() << "\nx_final:  " << xFinal.transpose() << std::endl;

  using cost_t = ocs2::double_integrator::DoubleIntegratorCost;
  std::unique_ptr<cost_t> cost(new cost_t(Q, R, QFinal));

  // cost desired trajectories
  const DIMENSIONS::scalar_t initTime(0.0);
  DIMENSIONS::scalar_t finalTime;
  ocs2::loadData::loadCppDataType(taskFile, "mpcTimeHorizon.timehorizon", finalTime);
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

  // partitioning times
  DIMENSIONS::scalar_array_t partitioningTimes{0.0, 1.0};

  // MPC ROS Node
  // ocs2::double_integrator::DoubleIntegratorInterface doubleIntegratorInterface(argv[1]);
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(taskFile);
  ocs2::PI_Settings piSettings;
  piSettings.loadSettings(taskFile);
  ocs2::Rollout_Settings rolloutSettings;
  rolloutSettings.loadSettings(taskFile, "pi.rollout");
  ocs2::TimeTriggeredRollout<STATE_DIM, INPUT_DIM> rollout(dynamics, rolloutSettings);

  ocs2::MPC_PI<STATE_DIM, INPUT_DIM> mpc_pi(&rollout, std::move(cost), constraint, partitioningTimes, mpcSettings, piSettings);
  mpc_pi.getSolverPtr()->setCostDesiredTrajectories(costDesiredTraj);
  ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM> mpcNode(mpc_pi, "double_integrator");

  mpcNode.launchNodes(argc, argv);

  // Successful exit
  return 0;
}
