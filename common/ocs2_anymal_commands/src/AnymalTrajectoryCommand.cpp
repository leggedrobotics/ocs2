/******************************************************************************
* File:             AnymalTrajectoryCommand.cpp
*
* Author:           Oliver Harley
* Created:          12/16/19
* Description:      AnymalTrajectoryCommand selects and runs the trajectories made from an external
*                   trajectory optimizer.
*****************************************************************************/


#include <ros/package.h>

#include <ocs2_scp_integration/ScpTrajectoryControlFwd.h>
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

  const auto defaultDisplacementVelocity = pt.get<scalar_t>("targetDisplacementVelocity");
  const auto defaultRotationVelocity = pt.get<scalar_t>("targetRotationVelocity");
  const scalar_t initCOMHeight = pt.get<scalar_t>("comHeight");

  Eigen::Matrix<scalar_t, switched_model::JOINT_COORDINATE_SIZE, 1> initJoints;
  ocs2::loadData::loadEigenMatrix(defaultTargetFile, "defaultJointState", initJoints);

/* Trajectory Command */
  switched_model::ScpTrajectoryControl<scalar_t>
    trajectoryCommand( argc, argv, "anymal", trajectoriesFile, initCOMHeight, initJoints, defaultRotationVelocity, defaultDisplacementVelocity, true);

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
