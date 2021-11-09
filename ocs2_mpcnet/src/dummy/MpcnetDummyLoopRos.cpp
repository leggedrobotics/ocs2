#include "ocs2_mpcnet/dummy/MpcnetDummyLoopRos.h"

#include <ros/ros.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcnetDummyLoopRos::MpcnetDummyLoopRos(scalar_t controlFrequency, scalar_t rosFrequency, std::unique_ptr<MpcnetControllerBase> mpcnetPtr,
                                       std::unique_ptr<RolloutBase> rolloutPtr, std::shared_ptr<RosReferenceManager> rosReferenceManagerPtr)
    : controlFrequency_(controlFrequency),
      rosFrequency_(rosFrequency),
      mpcnetPtr_(std::move(mpcnetPtr)),
      rolloutPtr_(std::move(rolloutPtr)),
      rosReferenceManagerPtr_(rosReferenceManagerPtr) {}

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
    scalar_array_t timeTrajectory;
    size_array_t postEventIndicesStock;
    vector_array_t stateTrajectory;
    vector_array_t inputTrajectory;
    rolloutPtr_->run(time, state, time + timeStep, mpcnetPtr_.get(), {}, timeTrajectory, postEventIndicesStock, stateTrajectory,
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

}  // namespace ocs2
