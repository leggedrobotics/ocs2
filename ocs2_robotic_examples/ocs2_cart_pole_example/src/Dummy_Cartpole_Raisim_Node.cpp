#include <ros/package.h>

#include <ocs2_oc/rollout/RaisimRollout.h>
#include <ocs2_cart_pole_example/CartPoleInterface.h>
#include <ocs2_cart_pole_example/definitions.h>
#include <ocs2_cart_pole_example/ros_comm/MRT_ROS_Cartpole.h>
#include <ocs2_cart_pole_example/ros_comm/MRT_ROS_Dummy_Cartpole.h>

int main(int argc, char* argv[]) {
  // task file
  if (argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
  std::string taskFileFolderName = std::string(argv[1]);

  std::cout << "Starting DUMMY" << std::endl;

  // setup MRT with simulator rollouts
  using sim_rollout_t = ocs2::RaisimRollout<ocs2::cartpole::STATE_DIM_, ocs2::cartpole::INPUT_DIM_>;
  std::unique_ptr<sim_rollout_t> simRollout(new sim_rollout_t(ros::package::getPath("ocs2_cart_pole_example") + "/urdf/cartpole.urdf"));
  ocs2::cartpole::MRT_ROS_Cartpole::Ptr mrtPtr(new ocs2::cartpole::MRT_ROS_Cartpole("cartpole"));
  mrtPtr->initRollout(std::move(simRollout));

  ocs2::cartpole::CartPoleInterface cartPoleInterface(taskFileFolderName);
  ocs2::cartpole::MRT_ROS_Dummy_Cartpole dummyCartpole(mrtPtr, cartPoleInterface.mpcSettings().mrtDesiredFrequency_,
                                                       cartPoleInterface.mpcSettings().mpcDesiredFrequency_);
  dummyCartpole.launchNodes(argc, argv);

  // initial state
  ocs2::SystemObservation<ocs2::cartpole::STATE_DIM_, ocs2::cartpole::INPUT_DIM_> initObservation;
  cartPoleInterface.getInitialState(initObservation.state());

  // initial command
  ocs2::CostDesiredTrajectories<double> initCostDesiredTrajectories(2);
  initCostDesiredTrajectories.desiredTimeTrajectory()[0] = 0.0;
  initCostDesiredTrajectories.desiredTimeTrajectory()[1] = 1.0;
  initCostDesiredTrajectories.desiredStateTrajectory()[0].setZero(ocs2::cartpole::STATE_DIM_);
  initCostDesiredTrajectories.desiredStateTrajectory()[1].setZero(ocs2::cartpole::STATE_DIM_);
  initCostDesiredTrajectories.desiredInputTrajectory()[0].setZero(ocs2::cartpole::INPUT_DIM_);
  initCostDesiredTrajectories.desiredInputTrajectory()[1].setZero(ocs2::cartpole::INPUT_DIM_);

  dummyCartpole.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
