#include "ocs2_mpcnet/rollout/MpcnetPolicyEvaluation.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcnetPolicyEvaluation::MetricsPtr MpcnetPolicyEvaluation::run(const std::string& policyFilePath, scalar_t timeStep,
                                                               const SystemObservation& initialObservation,
                                                               const ModeSchedule& modeSchedule,
                                                               const TargetTrajectories& targetTrajectories) {
  // declare metrics pointer
  MetricsPtr metricsPtr(new Metrics);

  // init time and state
  scalar_t time = initialObservation.time;
  vector_t state = initialObservation.state;

  // reset mpc
  mpcPtr_->reset();

  // reset rollout, i.e. reset the internal simulator state (e.g. relevant for RaiSim)
  rolloutPtr_->resetRollout();

  // prepare learned controller
  mpcnetPtr_->loadPolicyModel(policyFilePath);

  // update the reference manager
  referenceManagerPtr_->setModeSchedule(modeSchedule);
  referenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  // run policy evaluation
  int iteration = 0;
  try {
    while (time <= targetTrajectories.timeTrajectory.back()) {
      // run mpc and get solution
      if (!mpcPtr_->run(time, state)) {
        throw std::runtime_error("[MpcnetPolicyEvaluation::run] main routine of MPC returned false.");
      }
      PrimalSolution primalSolution = mpcPtr_->getSolverPtr()->primalSolution(mpcPtr_->getSolverPtr()->getFinalTime());

      // incurred quantities
      vector_t input = mpcnetPtr_->computeInput(time, state);
      metricsPtr->incurredHamiltonian += mpcPtr_->getSolverPtr()->getHamiltonian(time, state, input).f * timeStep;

      // forward simulate system with learned controller
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t stateTrajectory;
      vector_array_t inputTrajectory;
      rolloutPtr_->run(primalSolution.timeTrajectory_.front(), primalSolution.stateTrajectory_.front(),
                       primalSolution.timeTrajectory_.front() + timeStep, mpcnetPtr_.get(), primalSolution.modeSchedule_.eventTimes,
                       timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

      // update time, state and iteration
      time = timeTrajectory.back();
      state = stateTrajectory.back();
      ++iteration;

      // check if forward simulated system diverged
      if (!mpcnetDefinitionPtr_->validState(state)) {
        throw std::runtime_error("[MpcnetPolicyEvaluation::run] state is not valid.");
      }
    }
  } catch (const std::exception& e) {
    // print error for exceptions
    std::cerr << "[MpcnetPolicyEvaluation::run] a standard exception was caught, with message: " << e.what() << "\n";
    // this policy evaluation run failed, incurred quantities are not reported
    metricsPtr->incurredHamiltonian = std::numeric_limits<scalar_t>::quiet_NaN();
  }

  // report survival time
  metricsPtr->survivalTime = time;

  // return metrics pointer
  return metricsPtr;
}

}  // namespace ocs2
