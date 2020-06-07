#include <ros/package.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_raisim/RaisimRollout.h>
#include <ros/init.h>

#include <ocs2_cart_pole_example/CartPoleInterface.h>
#include <ocs2_cart_pole_example/definitions.h>
#include <ocs2_cart_pole_example/ros_comm/CartpoleDummyVisualization.h>
#include <ocs2_cart_pole_raisim_example/CartpoleRaisimConversions.h>

int main(int argc, char* argv[]) {
  const std::string robotName = "cartpole";

  // task file
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Robot interface
  ocs2::cartpole::CartPoleInterface cartPoleInterface(taskFileFolderName);

  // setup simulator rollouts
  std::unique_ptr<ocs2::RaisimRollout> simRollout(new ocs2::RaisimRollout(
      ocs2::cartpole::STATE_DIM_, ocs2::cartpole::INPUT_DIM_, ros::package::getPath("ocs2_cart_pole_example") + "/urdf/cartpole.urdf",
      &ocs2::cartpole::stateToRaisimGenCoordGenVel, &ocs2::cartpole::raisimGenCoordGenVelToState,
      &ocs2::cartpole::inputToRaisimGeneralizedForce));

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(simRollout.get());
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto cartpoleDummyVisualization = std::make_shared<ocs2::cartpole::CartpoleDummyVisualization>(nodeHandle);

  // Dummy loop
  ocs2::MRT_ROS_Dummy_Loop dummyCartpole(mrt, cartPoleInterface.mpcSettings().mrtDesiredFrequency_,
                                         cartPoleInterface.mpcSettings().mpcDesiredFrequency_);
  dummyCartpole.subscribeObservers({cartpoleDummyVisualization});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state() = cartPoleInterface.getInitialState();
  initObservation.input().setZero(ocs2::cartpole::INPUT_DIM_);
  initObservation.time() = 0;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({initObservation.time()}, {initObservation.state()}, {initObservation.input()});

  // Run dummy (loops while ros is ok)
  dummyCartpole.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
