//
// Created by ruben on 25.07.18.
//
#include <boost/filesystem.hpp>

#include <ocs2_anymal_loopshaping/asConstraint/definitions.h>
#include <ocs2_robotic_tools/command/TargetPoseTransformation.h>
#include <ocs2_quadruped_interface/test/MRT_ROS_Dummy_Quadruped.h>

#define SAVE_BAG

using namespace anymal;

int main( int argc, char* argv[] )
{
  const std::string robotName = "anymal";
  using MRT_ros_vis = switched_model::MRT_ROS_Dummy_Quadruped<
      anymal_loopshaping::JOINT_COORD_SIZE_,
      anymal_loopshaping::STATE_DIM_,
      anymal_loopshaping::INPUT_DIM_>;

  // Load task
  if ( argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
  boost::filesystem::path filePath(__FILE__);
  std::string pathToConfigFolder = filePath.parent_path().parent_path().parent_path().generic_string() + "/config/" + std::string(argv[1]);

  // Setup Interface, MRT, MPC
  OCS2AnymalLoopshapingInterface::Ptr optimizationInterfacePtr( new OCS2AnymalLoopshapingInterface(pathToConfigFolder));
  MRT_ROS_Anymal_Loopshaping mrt(optimizationInterfacePtr, pathToConfigFolder);

  MRT_ros_vis mrt_ros_vis(optimizationInterfacePtr,
                          mrt,
                          optimizationInterfacePtr->mpcSettings().mrtDesiredFrequency_,
                          robotName,
                          optimizationInterfacePtr->mpcSettings().mpcDesiredFrequency_);
  mrt_ros_vis.launchNodes(argc, argv);

  // initial state
  MRT_ROS_Anymal_Loopshaping::system_observation_t initObservation;
  initObservation.time()  = 0.0;
  initObservation.subsystem() = 15;
  initObservation.input().setZero();
  optimizationInterfacePtr->getInitialState(initObservation.state());

  // initial command
  MRT_ros_vis::cost_desired_trajectories_t initCostDesiredTrajectories;

  // time
  auto& timeTrajectory = initCostDesiredTrajectories.desiredTimeTrajectory();
  timeTrajectory.resize(1);
  timeTrajectory[0] = 0.0;

  // State
  auto& stateTrajectory = initCostDesiredTrajectories.desiredStateTrajectory();
  stateTrajectory.resize(1);
  stateTrajectory[0] = initObservation.state();

  // Input
  auto& inputTrajectory = initCostDesiredTrajectories.desiredInputTrajectory();
  inputTrajectory.resize(1);
  inputTrajectory[0] = initObservation.input();

  // run dummy
  mrt_ros_vis.run(initObservation, initCostDesiredTrajectories);

  return 0;
}
