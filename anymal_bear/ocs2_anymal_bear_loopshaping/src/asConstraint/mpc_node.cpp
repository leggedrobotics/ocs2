//
// Created by ruben on 25.07.18.
//

#include <chrono>
#include <iostream>
#include <string>

#include <ocs2_anymal_bear_loopshaping/asConstraint/definitions.h>
#include <boost/filesystem.hpp>

int main(int argc, char* argv[]) {
  // Time stamp
  std::time_t currentDate = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::cerr << "Current Time: " << std::ctime(&currentDate) << std::endl << std::endl;

  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  boost::filesystem::path filePath(__FILE__);
  std::string pathToConfigFolder = filePath.parent_path().parent_path().parent_path().generic_string() + "/config/" + std::string(argv[1]);

  anymal::OCS2AnymalLoopshapingInterface anymalInterface(pathToConfigFolder);

  // launch MPC nodes
  anymal::MPC_ROS_Anymal_Loopshaping ocs2AnymalMPC(anymalInterface.getMpc(), "anymal");
  ocs2AnymalMPC.launchNodes(argc, argv);

  return 0;
}
