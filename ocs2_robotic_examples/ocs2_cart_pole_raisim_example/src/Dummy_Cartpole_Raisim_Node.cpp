#include <ros/package.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_raisim/RaisimRollout.h>

#include <ocs2_cart_pole_example/CartPoleInterface.h>
#include <ocs2_cart_pole_example/definitions.h>
#include <ocs2_cart_pole_example/ros_comm/CartpoleDummyVisualization.h>
#include <ocs2_cart_pole_raisim_example/CartpoleRaisimConversions.h>

int main(int argc, char* argv[]) {
  const std::string robotName = "cartpole";
  using interface_t = ocs2::cartpole::CartPoleInterface;
  using vis_t = ocs2::cartpole::CartpoleDummyVisualization;
  using mrt_t = ocs2::MRT_ROS_Interface<ocs2::cartpole::STATE_DIM_, ocs2::cartpole::INPUT_DIM_>;
  using dummy_t = ocs2::MRT_ROS_Dummy_Loop<ocs2::cartpole::STATE_DIM_, ocs2::cartpole::INPUT_DIM_>;
  using sim_rollout_t = ocs2::RaisimRollout<ocs2::cartpole::STATE_DIM_, ocs2::cartpole::INPUT_DIM_>;

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Robot interface
  interface_t cartPoleInterface(taskFileFolderName);

  // setup simulator rollouts
  std::unique_ptr<sim_rollout_t> simRollout(new sim_rollout_t(
      ros::package::getPath("ocs2_cart_pole_example") + "/urdf/cartpole.urdf", &ocs2::cartpole::stateToRaisimGenCoordGenVel,
      &ocs2::cartpole::raisimGenCoordGenVelToState, &ocs2::cartpole::inputToRaisimGeneralizedForce));

  // MRT
  mrt_t mrt(robotName);
  mrt.initRollout(simRollout.get());
  mrt.launchNodes(nodeHandle);

  // Visualization
  std::shared_ptr<vis_t> cartpoleDummyVisualization(new vis_t(nodeHandle));

  // Dummy loop
  dummy_t dummyCartpole(mrt, cartPoleInterface.mpcSettings().mrtDesiredFrequency_, cartPoleInterface.mpcSettings().mpcDesiredFrequency_);
  dummyCartpole.subscribeObservers({cartpoleDummyVisualization});

  // initial state
  mrt_t::system_observation_t initObservation;
  initObservation.state() = cartPoleInterface.getInitialState();

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({initObservation.time()}, {initObservation.state()}, {initObservation.input()});

  // Run dummy (loops while ros is ok)
  dummyCartpole.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
