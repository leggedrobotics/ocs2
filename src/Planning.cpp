//
// Created by rgrandia on 11.02.19.
//

#include <string>
#include <iostream>

#include <ros/package.h>
#include <ocs2_quadruped_interface/QuadrupedXppVisualizer.h>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

int main( int argc, char* argv[] )
{
    // Time stamp
    std::time_t currentDate = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::cerr << "Current Time: " << std::ctime(&currentDate) << std::endl << std::endl;

    std::string pathToConfigFolder =  ros::package::getPath("ocs2_anymal_interface") + "/config/" + "planning";

    anymal::OCS2AnymalInterface::Ptr anymalInterface(new anymal::OCS2AnymalInterface(pathToConfigFolder));
    anymal::OCS2AnymalInterface::rbd_state_vector_t initialRbdState;
    initialRbdState.setZero();
    anymalInterface->runSLQ(0.0, initialRbdState, 1.6);

    anymal::OCS2AnymalInterface::eigen_scalar_array_t iterationCost, iterationISE1, iterationISE2;
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
    const std::vector<anymal::OCS2AnymalInterface::scalar_array_t>* timeTrajectoriesStockPtr;
    const anymal::OCS2AnymalInterface::state_vector_array2_t*	stateTrajectoriesStockPtr;
    const anymal::OCS2AnymalInterface::input_vector_array2_t* 	inputTrajectoriesStockPtr;
    anymalInterface->getOptimizedTrajectoriesPtr(timeTrajectoriesStockPtr, stateTrajectoriesStockPtr, inputTrajectoriesStockPtr);

    // Convert to observations
    typedef switched_model::QuadrupedXppVisualizer<12, 24, 24> visualizer_t;
    visualizer_t visualizer( anymalInterface , "anymal_planning" , true);
    visualizer.launchVisualizerNode(argc, argv);

    visualizer_t::system_observation_array_t observation_array;

    for (size_t i=0; i< timeTrajectoriesStockPtr->size(); i++){
        for (size_t k=0; k< timeTrajectoriesStockPtr->at(i).size(); k++){
            visualizer_t::system_observation_t observation;
            observation.time() = timeTrajectoriesStockPtr->at(i)[k];
            observation.state() = stateTrajectoriesStockPtr->at(i)[k];
            observation.input() = inputTrajectoriesStockPtr->at(i)[k];
            observation_array.push_back(observation);
        }
    }

    visualizer.publishTrajectory(observation_array, 0.25);

    return 0;
}

