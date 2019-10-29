//
// Created by ruben on 30.11.18.
//

#include <chrono>
#include <string>
#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>

#include <ocs2_anymal_bear_loopshaping/asConstraint/definitions.h>
#include <ocs2_quadruped_interface/QuadrupedXppVisualizer.h>

int main( int argc, char* argv[] )
{
  // Time stamp
  std::time_t currentDate = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::cerr << "Current Time: " << std::ctime(&currentDate) << std::endl << std::endl;

  boost::filesystem::path filePath(__FILE__);
  std::string pathToConfigFolder = filePath.parent_path().parent_path().parent_path().generic_string() + "/config/" + "planning";

  //
  anymal::OCS2AnymalLoopshapingInterface::Ptr anymalInterface(new anymal::OCS2AnymalLoopshapingInterface(pathToConfigFolder));
  anymal::OCS2AnymalLoopshapingInterface::rbd_state_vector_t initialRbdState;
  initialRbdState.setZero();
  anymalInterface->runSLQ(0.0, initialRbdState, 1.6);

  anymal::OCS2AnymalLoopshapingInterface::eigen_scalar_array_t iterationCost, iterationISE1, iterationISE2;
  anymalInterface->getIterationsLog(iterationCost, iterationISE1, iterationISE2);

  std::cerr << "Done" << std::endl << std::endl;

  std::cerr << "iterationCost_asConstraint = [";
  for (auto& v: iterationCost){
    std::cerr << v << ", ";
  }
  std::cerr << "]" << std::endl;

  std::cerr << "iterationISE1_asConstraint = [";
  for (auto& v: iterationISE1){
    std::cerr << v << ", ";
  }
  std::cerr << "]" << std::endl;

  std::cerr << "iterationISE2_asConstraint = [";
  for (auto& v: iterationISE2){
    std::cerr << v << ", ";
  }
  std::cerr << "]" << std::endl;

  // Get solution
  auto primalSolution = anymalInterface->getPrimalSolution();

  // Convert to observations
  typedef switched_model::QuadrupedXppVisualizer<12, 48, 48> visualizer_t;
  visualizer_t visualizer( anymalInterface , "anymal_loopshaping_planning" , true);
  visualizer.launchVisualizerNode(argc, argv);

  visualizer_t::system_observation_array_t observation_array;

  for (size_t k=0; k< primalSolution.timeTrajectory_.size(); k++){
    visualizer_t::system_observation_t observation;
    observation.time() = primalSolution.timeTrajectory_[k];
    observation.state() = primalSolution.stateTrajectory_[k];
    observation.input() = primalSolution.inputTrajectory_[k];
    observation_array.emplace_back(observation);
  }

  visualizer.publishTrajectory(observation_array, 0.25);

  // Print Feedback policy
  primalSolution.controllerPtr_->display();

  return 0;
}