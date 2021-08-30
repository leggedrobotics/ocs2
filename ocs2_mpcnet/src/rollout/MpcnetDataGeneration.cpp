#include "ocs2_mpcnet/rollout/MpcnetDataGeneration.h"

#include <random>

#include "ocs2_mpcnet/control/MpcnetBehavioralController.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcnetDataGeneration::DataPtr MpcnetDataGeneration::run(scalar_t alpha, const std::string& policyFilePath, scalar_t timeStep,
                                                        size_t dataDecimation, size_t nSamples, const matrix_t& samplingCovariance,
                                                        const SystemObservation& initialObservation, const ModeSchedule& modeSchedule,
                                                        const TargetTrajectories& targetTrajectories) {
  // declare data pointer
  DataPtr dataPtr(new DataArray);

  // init time and state
  scalar_t time = initialObservation.time;
  vector_t state = initialObservation.state;

  // reset mpc
  mpcPtr_->reset();

  // prepare learned controller
  mpcnetPtr_->loadPolicyModel(policyFilePath);

  // update the reference manager
  referenceManagerPtr_->setModeSchedule(modeSchedule);
  referenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  // set up behavioral controller with mixture parameter alpha and learned controller
  MpcnetBehavioralController behavioralController;
  behavioralController.setAlpha(alpha);
  behavioralController.setLearnedController(mpcnetPtr_->clone());

  // set up scalar standard normal generator and compute Cholesky decomposition of covariance matrix
  std::random_device randomDevice;
  std::default_random_engine pseudoRandomNumberGenerator(randomDevice());
  std::normal_distribution<scalar_t> standardNormalDistribution(scalar_t(0.0), scalar_t(1.0));
  std::function<scalar_t(scalar_t)> standardNormalNullaryOp = [&](scalar_t) -> scalar_t {
    return standardNormalDistribution(pseudoRandomNumberGenerator);
  };
  matrix_t S = samplingCovariance;
  matrix_t L = S.llt().matrixL();

  // run data generation
  int iteration = 0;
  try {
    while (time <= targetTrajectories.timeTrajectory.back()) {
      // run mpc and get solution
      if (!mpcPtr_->run(time, state)) {
        throw std::runtime_error("MpcnetDataGeneration::run Main routine of MPC returned false.");
      }
      PrimalSolution primalSolution = mpcPtr_->getSolverPtr()->primalSolution(mpcPtr_->getSolverPtr()->getFinalTime());

      // downsample the data signal by an integer factor
      if (iteration % dataDecimation == 0) {
        // get nominal data point
        {
          DataPoint dataPoint;
          dataPoint.t = primalSolution.timeTrajectory_[0];
          dataPoint.x = primalSolution.stateTrajectory_[0];
          dataPoint.u = primalSolution.controllerPtr_->computeInput(dataPoint.t, dataPoint.x);
          dataPoint.mode = primalSolution.modeSchedule_.modeAtTime(dataPoint.t);
          dataPoint.generalizedTime = mpcnetPtr_->getGeneralizedTime(dataPoint.t);
          dataPoint.relativeState = mpcnetPtr_->getRelativeState(dataPoint.t, dataPoint.x);
          dataPoint.hamiltonian = mpcPtr_->getSolverPtr()->getHamiltonian(dataPoint.t, dataPoint.x, dataPoint.u);
          dataPtr->push_back(std::move(dataPoint));
        }

        // get samples around nominal data point
        for (int i = 0; i < nSamples; i++) {
          DataPoint dataPoint;
          dataPoint.t = primalSolution.timeTrajectory_[0];
          dataPoint.x = primalSolution.stateTrajectory_[0] +
                        L * vector_t::NullaryExpr(primalSolution.stateTrajectory_[0].size(), standardNormalNullaryOp);
          dataPoint.u = primalSolution.controllerPtr_->computeInput(dataPoint.t, dataPoint.x);
          dataPoint.mode = primalSolution.modeSchedule_.modeAtTime(dataPoint.t);
          dataPoint.generalizedTime = mpcnetPtr_->getGeneralizedTime(dataPoint.t);
          dataPoint.relativeState = mpcnetPtr_->getRelativeState(dataPoint.t, dataPoint.x);
          dataPoint.hamiltonian = mpcPtr_->getSolverPtr()->getHamiltonian(dataPoint.t, dataPoint.x, dataPoint.u);
          dataPtr->push_back(std::move(dataPoint));
        }
      }

      // update behavioral controller with MPC controller
      behavioralController.setOptimalController(primalSolution.controllerPtr_->clone());

      // forward simulate system with behavioral controller
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t stateTrajectory;
      vector_array_t inputTrajectory;
      rolloutPtr_->run(primalSolution.timeTrajectory_[0], primalSolution.stateTrajectory_[0], primalSolution.timeTrajectory_[0] + timeStep,
                       &behavioralController, primalSolution.modeSchedule_.eventTimes, timeTrajectory, postEventIndicesStock,
                       stateTrajectory, inputTrajectory);

      // update time, state and iteration
      time = timeTrajectory.back();
      state = stateTrajectory.back();
      iteration++;

      // check if forward simulated system diverged
      if (!mpcnetDefinitionPtr_->validState(state)) {
        throw std::runtime_error("MpcnetDataGeneration::run State is not valid.");
      }
    }
  } catch (const std::exception& e) {
    // print error for exceptions
    std::cerr << "MpcnetDataGeneration::run A standard exception was caught, with message: " << e.what() << std::endl;
    // this data generation run failed, clear data
    dataPtr->clear();
  }

  // return data pointer
  return dataPtr;
}

}  // namespace ocs2
