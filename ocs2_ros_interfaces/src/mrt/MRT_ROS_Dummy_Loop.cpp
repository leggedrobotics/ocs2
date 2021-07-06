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

#include "ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Dummy_Loop::MRT_ROS_Dummy_Loop(MRT_ROS_Interface& mrt, scalar_t mrtDesiredFrequency, scalar_t mpcDesiredFrequency /*= -1*/)
    : mrt_(mrt),
      mrtDesiredFrequency_(mrtDesiredFrequency),
      mpcDesiredFrequency_(mpcDesiredFrequency),
      realtimeLoop_(mpcDesiredFrequency <= 0)  // true if mpcDesiredFrequency is not set or it is negative
{
  if (mrtDesiredFrequency_ < 0) {
    throw std::runtime_error("MRT loop frequency should be a positive number.");
  }

  if (mpcDesiredFrequency_ > 0) {
    ROS_WARN_STREAM("MPC loop is not realtime! For realtime setting, set mpcDesiredFrequency to any negative number.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MRT_ROS_Dummy_Loop::run(const SystemObservation& initObservation, const TargetTrajectories& initTargetTrajectories) {
  ros::WallRate rosRate(mrtDesiredFrequency_);  // in Hz

  // time step
  const scalar_t timeStep = (1.0 / mrtDesiredFrequency_);

  // set the frequency ratio between MRT loop and MPC loop in the case of non realtime test
  size_t frequencyRatio = 1;
  if (!realtimeLoop_) {
    frequencyRatio = static_cast<size_t>(mrtDesiredFrequency_ / mpcDesiredFrequency_);
  }

  size_t loopCounter = 0;
  scalar_t time = initObservation.time;

  // reset MPC node
  mrt_.resetMpcNode(initTargetTrajectories);

  // wait for the initial MPC plan
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (::ros::ok() && ::ros::master::check()) {
    mrt_.spinMRT();
    // for initial plan
    mrt_.setCurrentObservation(initObservation);
    if (mrt_.initialPolicyReceived()) {
      break;
    } else {
      ::ros::WallDuration(timeStep).sleep();
    }
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  observation_ = initObservation;

  while (::ros::ok() && ::ros::master::check()) {
    // this should be called before updatePolicy()
    mrt_.spinMRT();

    // Checks for new policy and updates the policy
    bool policyUpdated = false;
    if (realtimeLoop_) {
      policyUpdated = mrt_.updatePolicy();

    } else if (loopCounter % frequencyRatio == 0) {
      while (::ros::ok() && ::ros::master::check()) {
        policyUpdated = mrt_.updatePolicy();
        if (policyUpdated) {
          break;
        } else {
          mrt_.spinMRT();
        }
      }
      std::cout << "<<< Message received at " << time << "\n";
    }

    std::cout << "### Current time " << time << "\n";

    // integrate nominal dynamics if available, otherwise fake simulation
    vector_t stateTemp = observation_.state;
    if (mrt_.isRolloutSet()) {
      mrt_.rolloutPolicy(time, stateTemp, timeStep, observation_.state, observation_.input, observation_.mode);
    } else {
      mrt_.evaluatePolicy(time + timeStep, stateTemp, observation_.state, observation_.input, observation_.mode);
    }

    // time and loop counter increment
    loopCounter++;
    time += timeStep;
    observation_.time = time;

    // user-defined modifications before publishing
    modifyObservation(observation_);

    // publish observation
    if (realtimeLoop_) {
      mrt_.setCurrentObservation(observation_);
    } else if (loopCounter % frequencyRatio == 0) {
      mrt_.setCurrentObservation(observation_);
      std::cout << ">>> Observation is published at " << time << "\n";
    }

    // Update observers
    for (auto& observer : observers_) {
      observer->update(observation_, mrt_.getPolicy(), mrt_.getCommand());
    }

    ros::spinOnce();
    rosRate.sleep();
  }  // end of while loop
}

}  // namespace ocs2
