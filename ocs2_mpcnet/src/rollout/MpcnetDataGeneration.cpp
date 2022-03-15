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

  // reset rollout, i.e. reset the internal simulator state (e.g. relevant for RaiSim)
  rolloutPtr_->resetRollout();

  // prepare learned controller
  mpcnetPtr_->loadPolicyModel(policyFilePath);

  // update the reference manager
  referenceManagerPtr_->setModeSchedule(modeSchedule);
  referenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  // set up behavioral controller with mixture parameter alpha and learned controller
  std::unique_ptr<MpcnetBehavioralController> behavioralControllerPtr;
  behavioralControllerPtr->setAlpha(alpha);
  behavioralControllerPtr->setLearnedController(*mpcnetPtr_);

  // set up scalar standard normal generator and compute Cholesky decomposition of covariance matrix
  std::random_device randomDevice;
  std::default_random_engine pseudoRandomNumberGenerator(randomDevice());
  std::normal_distribution<scalar_t> standardNormalDistribution(scalar_t(0.0), scalar_t(1.0));
  auto standardNormalNullaryOp = [&](scalar_t) -> scalar_t { return standardNormalDistribution(pseudoRandomNumberGenerator); };
  const matrix_t L = samplingCovariance.llt().matrixL();

  // run data generation
  int iteration = 0;
  try {
    while (time <= targetTrajectories.timeTrajectory.back()) {
      // run mpc and get solution
      if (!mpcPtr_->run(time, state)) {
        throw std::runtime_error("[MpcnetDataGeneration::run] main routine of MPC returned false.");
      }
      const auto primalSolution = mpcPtr_->getSolverPtr()->primalSolution(mpcPtr_->getSolverPtr()->getFinalTime());

      // downsample the data signal by an integer factor
      if (iteration % dataDecimation == 0) {
        // get nominal data point
        {
          DataPoint dataPoint;
          dataPoint.t = primalSolution.timeTrajectory_.front();
          dataPoint.x = primalSolution.stateTrajectory_.front();
          dataPoint.u = primalSolution.controllerPtr_->computeInput(dataPoint.t, dataPoint.x);
          dataPoint.mode = primalSolution.modeSchedule_.modeAtTime(dataPoint.t);
          dataPoint.generalizedTime = mpcnetDefinitionPtr_->getGeneralizedTime(dataPoint.t, referenceManagerPtr_->getModeSchedule());
          dataPoint.relativeState =
              mpcnetDefinitionPtr_->getRelativeState(dataPoint.t, dataPoint.x, referenceManagerPtr_->getTargetTrajectories());
          dataPoint.inputTransformation = mpcnetDefinitionPtr_->getInputTransformation(dataPoint.t, dataPoint.x);
          dataPoint.hamiltonian = mpcPtr_->getSolverPtr()->getHamiltonian(dataPoint.t, dataPoint.x, dataPoint.u);
          dataPtr->push_back(std::move(dataPoint));
        }

        // get samples around nominal data point
        for (int i = 0; i < nSamples; i++) {
          DataPoint dataPoint;
          dataPoint.t = primalSolution.timeTrajectory_.front();
          dataPoint.x = primalSolution.stateTrajectory_.front();
          dataPoint.x.noalias() += L * vector_t::NullaryExpr(primalSolution.stateTrajectory_.front().size(), standardNormalNullaryOp);
          dataPoint.u = primalSolution.controllerPtr_->computeInput(dataPoint.t, dataPoint.x);
          dataPoint.mode = primalSolution.modeSchedule_.modeAtTime(dataPoint.t);
          dataPoint.generalizedTime = mpcnetDefinitionPtr_->getGeneralizedTime(dataPoint.t, referenceManagerPtr_->getModeSchedule());
          dataPoint.relativeState =
              mpcnetDefinitionPtr_->getRelativeState(dataPoint.t, dataPoint.x, referenceManagerPtr_->getTargetTrajectories());
          dataPoint.inputTransformation = mpcnetDefinitionPtr_->getInputTransformation(dataPoint.t, dataPoint.x);
          dataPoint.hamiltonian = mpcPtr_->getSolverPtr()->getHamiltonian(dataPoint.t, dataPoint.x, dataPoint.u);
          dataPtr->push_back(std::move(dataPoint));
        }
      }

      // update behavioral controller with MPC controller
      behavioralControllerPtr->setOptimalController(*primalSolution.controllerPtr_);

      // forward simulate system with behavioral controller
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t stateTrajectory;
      vector_array_t inputTrajectory;
      rolloutPtr_->run(primalSolution.timeTrajectory_.front(), primalSolution.stateTrajectory_.front(),
                       primalSolution.timeTrajectory_.front() + timeStep, behavioralControllerPtr.get(),
                       primalSolution.modeSchedule_.eventTimes, timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

      // update time, state and iteration
      time = timeTrajectory.back();
      state = stateTrajectory.back();
      ++iteration;

      // check if forward simulated system diverged
      if (!mpcnetDefinitionPtr_->validState(state)) {
        throw std::runtime_error("[MpcnetDataGeneration::run] state is not valid.");
      }
    }
  } catch (const std::exception& e) {
    // print error for exceptions
    std::cerr << "[MpcnetDataGeneration::run] a standard exception was caught, with message: " << e.what() << "\n";
    // this data generation run failed, clear data
    dataPtr->clear();
  }

  // return data pointer
  return dataPtr;
}

}  // namespace ocs2
