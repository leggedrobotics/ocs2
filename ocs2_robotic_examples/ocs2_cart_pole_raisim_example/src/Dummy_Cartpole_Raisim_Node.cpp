#include <ros/package.h>

#include <ocs2_cart_pole_example/CartPoleInterface.h>
#include <ocs2_cart_pole_example/definitions.h>
#include <ocs2_cart_pole_example/ros_comm/MRT_ROS_Dummy_Cartpole.h>
#include <ocs2_cart_pole_raisim_example/CartpoleRaisimConversions.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_raisim/RaisimRollout.h>

int main(int argc, char* argv[]) {
  using ocs2::cartpole::INPUT_DIM_;
  using ocs2::cartpole::STATE_DIM_;
  using state_vector_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::state_vector_t;
  using input_vector_t = ocs2::Dimensions<STATE_DIM_, INPUT_DIM_>::input_vector_t;

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // setup MRT with simulator rollouts
  using sim_rollout_t = ocs2::RaisimRollout<STATE_DIM_, INPUT_DIM_>;
  std::unique_ptr<sim_rollout_t> simRollout(new sim_rollout_t(
      ros::package::getPath("ocs2_cart_pole_example") + "/urdf/cartpole.urdf", &ocs2::cartpole::stateToRaisimGenCoordGenVel,
      &ocs2::cartpole::raisimGenCoordGenVelToState, &ocs2::cartpole::inputToRaisimGeneralizedForce));
  ocs2::MRT_ROS_Interface<STATE_DIM_, INPUT_DIM_> mrt("cartpole");
  mrt.initRollout(simRollout.get());

  ocs2::cartpole::CartPoleInterface cartPoleInterface(taskFileFolderName);
  ocs2::cartpole::MrtRosDummyCartpole dummyCartpole(mrt, cartPoleInterface.mpcSettings().mrtDesiredFrequency_,
                                                    cartPoleInterface.mpcSettings().mpcDesiredFrequency_);
  dummyCartpole.launchNodes(argc, argv);

  // initial state
  ocs2::SystemObservation<STATE_DIM_, INPUT_DIM_> initObservation;
  cartPoleInterface.getInitialState(initObservation.state());

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories(2);
  initCostDesiredTrajectories.desiredTimeTrajectory()[0] = 0.0;
  initCostDesiredTrajectories.desiredTimeTrajectory()[1] = 1.0;
  initCostDesiredTrajectories.desiredStateTrajectory()[0].setZero(STATE_DIM_);
  initCostDesiredTrajectories.desiredStateTrajectory()[1].setZero(STATE_DIM_);
  initCostDesiredTrajectories.desiredInputTrajectory()[0].setZero(INPUT_DIM_);
  initCostDesiredTrajectories.desiredInputTrajectory()[1].setZero(INPUT_DIM_);

  dummyCartpole.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
