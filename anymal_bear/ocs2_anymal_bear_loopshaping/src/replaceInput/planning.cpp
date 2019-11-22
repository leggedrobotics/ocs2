//
// Created by ruben on 30.11.18.
//

#include <chrono>
#include <string>
#include <iostream>

#include <boost/filesystem.hpp>

#include <ocs2_anymal_bear_loopshaping/replaceInput/definitions.h>

int main( int argc, char* argv[] )
{
  // Time stamp
  std::time_t currentDate = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::cerr << "Current Time: " << std::ctime(&currentDate) << std::endl << std::endl;

  boost::filesystem::path filePath(__FILE__);
  std::string pathToConfigFolder = filePath.parent_path().parent_path().parent_path().generic_string() + "/config/" + "planning";

  //
  anymal::OCS2AnymalLoopshapingInterface anymalInterface(pathToConfigFolder);
  anymal::OCS2AnymalLoopshapingInterface::rbd_state_vector_t initialRbdState;
  initialRbdState.setZero();
  anymalInterface.runSLQ(0.0, initialRbdState, 1.6);

  anymal::OCS2AnymalLoopshapingInterface::eigen_scalar_array_t iterationCost, iterationISE1, iterationISE2;
  anymalInterface.getIterationsLog(iterationCost, iterationISE1, iterationISE2);

  std::cerr << "Done" << std::endl << std::endl;

  std::cerr << "iterationCost_replaceInput = [";
  for (auto& v: iterationCost){
    std::cerr << v << ", ";
  }
  std::cerr << "]" << std::endl;

  std::cerr << "iterationISE1_replaceInput = [";
  for (auto& v: iterationISE1){
    std::cerr << v << ", ";
  }
  std::cerr << "]" << std::endl;

  std::cerr << "iterationISE2_replaceInput = [";
  for (auto& v: iterationISE2){
    std::cerr << v << ", ";
  }
  std::cerr << "]" << std::endl;

  return 0;
}