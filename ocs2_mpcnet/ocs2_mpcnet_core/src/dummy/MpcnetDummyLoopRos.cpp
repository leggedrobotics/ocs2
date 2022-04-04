/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include "ocs2_mpcnet_core/dummy/MpcnetDummyLoopRos.h"

#include <ros/ros.h>

namespace ocs2 {
namespace mpcnet {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcnetDummyLoopRos::MpcnetDummyLoopRos(scalar_t controlFrequency, scalar_t rosFrequency, std::unique_ptr<MpcnetControllerBase> mpcnetPtr,
                                       std::unique_ptr<RolloutBase> rolloutPtr, std::shared_ptr<RosReferenceManager> rosReferenceManagerPtr)
    : controlFrequency_(controlFrequency),
      rosFrequency_(rosFrequency),
      mpcnetPtr_(std::move(mpcnetPtr)),
      rolloutPtr_(std::move(rolloutPtr)),
      rosReferenceManagerPtr_(std::move(rosReferenceManagerPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetDummyLoopRos::run(const SystemObservation& systemObservation, const TargetTrajectories& targetTrajectories) {
  ros::WallRate rosRate(rosFrequency_);
  scalar_t duration = 1.0 / rosFrequency_;

  // initialize
  SystemObservation initialSystemObservation = systemObservation;
  SystemObservation finalSystemObservation = systemObservation;
  rosReferenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  // start of while loop
  while (::ros::ok() && ::ros::master::check()) {
    // update system observation
    swap(initialSystemObservation, finalSystemObservation);

    // update reference manager and synchronized modules
    preSolverRun(initialSystemObservation.time, initialSystemObservation.state);

    // rollout
    rollout(duration, initialSystemObservation, finalSystemObservation);

    // update observers
    PrimalSolution primalSolution;
    primalSolution.timeTrajectory_ = {finalSystemObservation.time};
    primalSolution.stateTrajectory_ = {finalSystemObservation.state};
    primalSolution.inputTrajectory_ = {finalSystemObservation.input};
    primalSolution.modeSchedule_ = rosReferenceManagerPtr_->getModeSchedule();
    primalSolution.controllerPtr_ = std::unique_ptr<ControllerBase>(mpcnetPtr_->clone());
    CommandData commandData;
    commandData.mpcInitObservation_ = initialSystemObservation;
    commandData.mpcTargetTrajectories_ = rosReferenceManagerPtr_->getTargetTrajectories();
    for (auto& observer : observerPtrs_) {
      observer->update(finalSystemObservation, primalSolution, commandData);
    }

    // process callbacks and sleep
    ::ros::spinOnce();
    rosRate.sleep();
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetDummyLoopRos::addObserver(std::shared_ptr<DummyObserver> observer) {
  observerPtrs_.push_back(std::move(observer));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetDummyLoopRos::addSynchronizedModule(std::shared_ptr<SolverSynchronizedModule> synchronizedModule) {
  synchronizedModulePtrs_.push_back(std::move(synchronizedModule));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetDummyLoopRos::rollout(scalar_t duration, const SystemObservation& initialSystemObservation,
                                 SystemObservation& finalSystemObservation) {
  scalar_t timeStep = 1.0 / controlFrequency_;

  // initial time, state and input
  scalar_t time = initialSystemObservation.time;
  vector_t state = initialSystemObservation.state;
  vector_t input = initialSystemObservation.input;

  // start of while loop
  while (time <= initialSystemObservation.time + duration) {
    // forward simulate system
    ModeSchedule modeSchedule = rosReferenceManagerPtr_->getModeSchedule();
    scalar_array_t timeTrajectory;
    size_array_t postEventIndicesStock;
    vector_array_t stateTrajectory;
    vector_array_t inputTrajectory;
    rolloutPtr_->run(time, state, time + timeStep, mpcnetPtr_.get(), modeSchedule, timeTrajectory, postEventIndicesStock, stateTrajectory,
                     inputTrajectory);

    // update time, state and input
    time = timeTrajectory.back();
    state = stateTrajectory.back();
    input = inputTrajectory.back();
  }  // end of while loop

  // final time, state and input
  finalSystemObservation.time = time;
  finalSystemObservation.state = state;
  finalSystemObservation.input = input;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetDummyLoopRos::preSolverRun(scalar_t time, const vector_t& state) {
  rosReferenceManagerPtr_->preSolverRun(time, time + scalar_t(1.0), state);
  for (auto& module : synchronizedModulePtrs_) {
    module->preSolverRun(time, time + scalar_t(1.0), state, *rosReferenceManagerPtr_);
  }
}

}  // namespace mpcnet
}  // namespace ocs2
