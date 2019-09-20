#include <ros/package.h>

#include <ocs2_cart_pole_example/CartPoleInterface.h>
#include <ocs2_cart_pole_example/definitions.h>
#include <ocs2_cart_pole_example/ros_comm/MRT_ROS_Dummy_Cartpole.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_oc/rollout/RaisimRollout.h>

using ocs2::cartpole::INPUT_DIM_;
using ocs2::cartpole::STATE_DIM_;
using state_vector_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::state_vector_t;
using input_vector_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::input_vector_t;

/**
 * @brief Convert ocs2 cartpole state to generalized coordinate and generalized velocity used by RAIsim
 * @param[in] state: the state to be converted
 * @return {q, dq} pair that represents the state
 */
std::pair<Eigen::VectorXd, Eigen::VectorXd> stateToRaisimGenCoordGenVel(const state_vector_t& state, const input_vector_t&) {
  return {state.head<2>(), state.tail<2>()};
}

/**
 * @brief Convert RAIsim generalized coordinates and velocities to ocs2 cartpole state
 * @note This should be the inverse to stateToRaisimGenCoordGenVel
 * @param[in] q the generalized coordinate
 * @param[in] dq the generalized velocity
 * @return the corresponding ocs2 cart pole state
 */
ocs2::Dimensions<ocs2::cartpole::STATE_DIM_, ocs2::cartpole::INPUT_DIM_>::state_vector_t raisimGenCoordGenVelToState(
    const Eigen::VectorXd& q, const Eigen::VectorXd& dq) {
  state_vector_t state;
  state << q, dq;
  return state;
}

/**
 * @brief Convert ocs2 control input to RAIsim generalized force
 * @param[in] input: The control computed by the ocs2 controller
 * @param[in] state: The current state
 * @return The generalized forces to be applied to the system
 */
Eigen::VectorXd inputToRaisimGeneralizedForce(double, const input_vector_t& input, const state_vector_t&, const Eigen::VectorXd&,
                                              const Eigen::VectorXd&) {
  Eigen::VectorXd generalizedForce = Eigen::VectorXd::Zero(2);
  generalizedForce.head(1) = input;
  return generalizedForce;
}

int main(int argc, char* argv[]) {
  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // setup MRT with simulator rollouts
  using sim_rollout_t = ocs2::RaisimRollout<STATE_DIM_, INPUT_DIM_>;
  std::unique_ptr<sim_rollout_t> simRollout(new sim_rollout_t(ros::package::getPath("ocs2_cart_pole_example") + "/urdf/cartpole.urdf",
                                                              &stateToRaisimGenCoordGenVel, &raisimGenCoordGenVelToState,
                                                              &inputToRaisimGeneralizedForce));
  ocs2::MRT_ROS_Interface<STATE_DIM_, INPUT_DIM_> mrt("cartpole");
  mrt.initRollout(std::move(simRollout));

  ocs2::cartpole::CartPoleInterface cartPoleInterface(taskFileFolderName);
  ocs2::cartpole::MrtRosDummyCartpole dummyCartpole(mrt, cartPoleInterface.mpcSettings().mrtDesiredFrequency_,
                                                    cartPoleInterface.mpcSettings().mpcDesiredFrequency_);
  dummyCartpole.launchNodes(argc, argv);

  // initial state
  ocs2::SystemObservation<STATE_DIM_, INPUT_DIM_> initObservation;
  cartPoleInterface.getInitialState(initObservation.state());

  // initial command
  ocs2::CostDesiredTrajectories<double> initCostDesiredTrajectories(2);
  initCostDesiredTrajectories.desiredTimeTrajectory()[0] = 0.0;
  initCostDesiredTrajectories.desiredTimeTrajectory()[1] = 1.0;
  initCostDesiredTrajectories.desiredStateTrajectory()[0].setZero(STATE_DIM_);
  initCostDesiredTrajectories.desiredStateTrajectory()[1].setZero(STATE_DIM_);
  initCostDesiredTrajectories.desiredInputTrajectory()[0].setZero(INPUT_DIM_);
  initCostDesiredTrajectories.desiredInputTrajectory()[1].setZero(INPUT_DIM_);

  dummyCartpole.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
