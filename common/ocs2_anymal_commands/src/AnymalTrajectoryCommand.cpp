/******************************************************************************
* File:             AnymalTrajectoryCommand.cpp
*
* Author:           Oliver Harley
* Created:          12/16/19
* Description:      AnymalTrajectoryCommand selects and runs the trajectories made from an external
*                   trajectory optimizer.
*****************************************************************************/


#include <ros/package.h>

#include <ocs2_scp_integration/implementation/ScpTrajectoryControl.hpp>
#include <ocs2_core/misc/LoadData.h>

int main(int argc, char* argv[]) {
  using scalar_t = double;
  std::string trajectoriesFile = ros::package::getPath("ocs2_scp_integration") + "/config/trajectories.info";
  std::cerr << "Loading trajectories file: " << trajectoriesFile << std::endl;

  // Load the initial state
  std::string defaultTargetFile = ros::package::getPath("ocs2_anymal_commands") + "/config/targetCommand.info";

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(defaultTargetFile, pt);

  const auto targetDisplacementVelocity = pt.get<scalar_t>("targetDisplacementVelocity");
  const auto targetRotationVelocity = pt.get<scalar_t>("targetRotationVelocity");
  const scalar_t initZHeight = pt.get<scalar_t>("comHeight");

  Eigen::Matrix<scalar_t, switched_model::JOINT_COORDINATE_SIZE, 1> initJoints;
  ocs2::loadData::loadEigenMatrix(defaultTargetFile, "defaultJointState", initJoints);

  // Set the initial state with the target trajectories keyboard interface
  /* TargetPoseCommand */
  //TODO(oharley): may have to inline, could be useful for resetting after a trajectory
  // switched_model::TargetTrajectories_Keyboard_Quadruped<double, 24, 24>
  // targetPoseCommand( argc, argv, "anymal", initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity);
  //
  // const std::string commadMsg = "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces";
  // targetPoseCommand.getKeyboardCommand(commadMsg);

/* ModeSequence Command */
  // switched_model::ModeSequence_Keyboard_Quadruped<double> modeSequenceCommand(gaitFile, "anymal", true);
  // modeSequenceCommand.launchNodes(argc, argv);
  // modeSequenceCommand.getKeyboardCommand();

/* Trajectory Command */
  switched_model::ScpTrajectoryControl<scalar_t>
    trajectoryCommand( argc, argv, "anymal", trajectoriesFile, initZHeight, initJoints, false);


  /************************************************************************************************
  *                                         launch node                                          *
  ************************************************************************************************/

  //ModeSequencCommand seems to take argc, and argv, the rest do not
  trajectoryCommand.launchNodes(argc, argv);

  const std::string commandMsg = "Enter trajectory number, for the list of available trajectories enter 'list'";
  trajectoryCommand.getKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
