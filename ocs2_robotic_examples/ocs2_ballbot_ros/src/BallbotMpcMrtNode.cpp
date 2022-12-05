/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/*
 * This file contains on an example on how to integrate the OCS2 MPC into your own control loop.
 * In contrast to the MPC-node + Dummy-node setup, in this case we will not use ROS to communicate between the MPC and MRT.
 * The MPC will run in a separate thread, and the ocs2::MPC_MRT_Interface facilitates all communication with this thread.
 *
 * This removes latency that arises when trying to do high frequency control over ROS.
 * ROS will only be used to send commands, and to publish visualization topics.
 */

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include <ocs2_ballbot/BallbotInterface.h>
#include "ocs2_ballbot_ros/BallbotDummyVisualization.h"

/**
 * This function implements the evaluation of the MPC policy
 * @param currentObservation : current system observation {time, state, input} to compute the input for. (input can be left empty)
 * @param mpcMrtInterface : interface used for communication with the MPC optimization (running in a different thread)
 * @return system input u(t)
 */
ocs2::vector_t mpcTrackingController(const ocs2::SystemObservation& currentObservation, ocs2::MPC_MRT_Interface& mpcMrtInterface);

int main(int argc, char** argv) {
  /*
   * Load the robot specific problem specification
   */
  const std::string robotName = "ballbot";

  // task file
  std::vector<std::string> programArgs{};
  ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc_mrt");
  ros::NodeHandle nodeHandle;

  // Robot interface
  const std::string taskFile = ros::package::getPath("ocs2_ballbot") + "/config/" + taskFileFolderName + "/task.info";
  const std::string libFolder = ros::package::getPath("ocs2_ballbot") + "/auto_generated";
  ocs2::ballbot::BallbotInterface ballbotInterface(taskFile, libFolder);

  /*
   * Set up the MPC and the MPC_MRT_interface.
   * For this example we add a command interface and a visualization which both communicate over ros
   */

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(ballbotInterface.mpcSettings(), ballbotInterface.ddpSettings(), ballbotInterface.getRollout(),
                               ballbotInterface.getOptimalControlProblem(), ballbotInterface.getInitializer());

  // ROS ReferenceManager. This gives us the command interface. Requires the observations to be published
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, ballbotInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);
  auto observationPublisher = nodeHandle.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // Visualization
  ocs2::ballbot::BallbotDummyVisualization ballbotDummyVisualization(nodeHandle);

  // Create the MPC MRT Interface
  ocs2::MPC_MRT_Interface mpcMrtInterface(mpc);
  mpcMrtInterface.initRollout(&ballbotInterface.getRollout());

  /*
   * Initialize the simulation and controller
   */

  // Initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = ballbotInterface.getInitialState();
  initObservation.input.setZero(ocs2::ballbot::INPUT_DIM);
  initObservation.time = 0.0;

  // Initial command
  const ocs2::TargetTrajectories initTargetTrajectories({initObservation.time}, {initObservation.state}, {initObservation.input});

  // Set the first observation and command and wait for optimization to finish
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  mpcMrtInterface.setCurrentObservation(initObservation);
  mpcMrtInterface.getReferenceManager().setTargetTrajectories(initTargetTrajectories);
  while (!mpcMrtInterface.initialPolicyReceived() && ros::ok() && ros::master::check()) {
    mpcMrtInterface.advanceMpc();
    ros::WallRate(ballbotInterface.mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  /*
   * Launch the computation of the MPC in a separate thread.
   * This thread will be triggered at a given frequency and execute an optimization based on the latest available observation.
   */
  std::atomic_bool mpcRunning{true};
  auto mpcThread = std::thread([&]() {
    while (mpcRunning) {
      try {
        ocs2::executeAndSleep([&]() { mpcMrtInterface.advanceMpc(); }, ballbotInterface.mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        mpcRunning = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
      }
    }
  });
  ocs2::setThreadPriority(ballbotInterface.ddpSettings().threadPriority_, mpcThread);

  /*
   * Main control loop.
   */
  ocs2::SystemObservation currentObservation = initObservation;
  while (mpcRunning && ros::ok()) {
    ocs2::executeAndSleep(  // timed execution of the control loop
        [&]() {
          ROS_INFO_STREAM("### Current time " << currentObservation.time);

          /*
           * State estimation would go here to fill "currentObservation".
           * In this example we receive the measurement directly after forward integration at the end of the loop.
           */

          // Evaluate the control input
          const auto systemInput = mpcTrackingController(currentObservation, mpcMrtInterface);

          /*
           * Sending the commands to the actuators would go here.
           * In this example, we instead do a forward simulation + visualization.
           * Simulation is done with the rollout functionality of the mpcMrtInterface, but this can be replaced by any other simulation.
           */

          // Forward simulation
          const auto dt = 1.0 / ballbotInterface.mpcSettings().mrtDesiredFrequency_;
          ocs2::SystemObservation nextObservation;
          nextObservation.time = currentObservation.time + dt;
          mpcMrtInterface.rolloutPolicy(currentObservation.time, currentObservation.state, dt, nextObservation.state, nextObservation.input,
                                        nextObservation.mode);

          // "state estimation"
          currentObservation = nextObservation;

          // Visualization
          ballbotDummyVisualization.update(currentObservation, mpcMrtInterface.getPolicy(), mpcMrtInterface.getCommand());

          // Publish the observation. Only needed for the command interface
          observationPublisher.publish(ocs2::ros_msg_conversions::createObservationMsg(currentObservation));

          ros::spinOnce();
        },
        ballbotInterface.mpcSettings().mrtDesiredFrequency_);
  }

  // Shut down the MPC thread.
  mpcRunning = false;
  if (mpcThread.joinable()) {
    mpcThread.join();
  }

  // Successful exit
  return 0;
}

ocs2::vector_t mpcTrackingController(const ocs2::SystemObservation& currentObservation, ocs2::MPC_MRT_Interface& mpcMrtInterface) {
  // Update the current state of the system
  mpcMrtInterface.setCurrentObservation(currentObservation);

  // Load the latest MPC policy
  bool policyUpdated = mpcMrtInterface.updatePolicy();
  if (policyUpdated) {
    ROS_INFO_STREAM("<<< New MPC policy received at " << currentObservation.time);
  }

  // Evaluate the current policy
  ocs2::vector_t optimizedState;  // Evaluation of the optimized state trajectory.
  ocs2::vector_t optimizedInput;  // Evaluation of the optimized input trajectory.
  size_t plannedMode;             // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface.evaluatePolicy(currentObservation.time, currentObservation.state, optimizedState, optimizedInput, plannedMode);

  return optimizedInput;
}
