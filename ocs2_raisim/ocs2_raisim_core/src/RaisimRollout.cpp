/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_raisim_core/RaisimRollout.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RaisimRollout::RaisimRollout(std::string urdfFile, std::string resourcePath,
                             state_to_raisim_gen_coord_gen_vel_t stateToRaisimGenCoordGenVel,
                             raisim_gen_coord_gen_vel_to_state_t raisimGenCoordGenVelToState,
                             input_to_raisim_generalized_force_t inputToRaisimGeneralizedForce,
                             data_extraction_callback_t dataExtractionCallback, RaisimRolloutSettings raisimRolloutSettings,
                             input_to_raisim_pd_targets_t inputToRaisimPdTargets)
    : RolloutBase(raisimRolloutSettings.rolloutSettings_),
      raisimRolloutSettings_(std::move(raisimRolloutSettings)),
      urdfFile_(std::move(urdfFile)),
      resourcePath_(std::move(resourcePath)),
      ground_(nullptr),
      system_(nullptr),
      stateToRaisimGenCoordGenVel_(std::move(stateToRaisimGenCoordGenVel)),
      raisimGenCoordGenVelToState_(std::move(raisimGenCoordGenVelToState)),
      inputToRaisimGeneralizedForce_(std::move(inputToRaisimGeneralizedForce)),
      dataExtractionCallback_(std::move(dataExtractionCallback)),
      inputToRaisimPdTargets_(std::move(inputToRaisimPdTargets)) {
  world_.setTimeStep(this->settings().timeStep);

  system_ = world_.addArticulatedSystem(urdfFile_, resourcePath_, raisimRolloutSettings_.orderedJointNames_);
  system_->setControlMode(raisimRolloutSettings_.controlMode_);
  if (raisimRolloutSettings_.controlMode_ != raisim::ControlMode::FORCE_AND_TORQUE) {
    system_->setPdGains(raisimRolloutSettings_.pGains_, raisimRolloutSettings_.dGains_);
  }

  std::cerr << "\nInstantiated Raisim System with DoF = " << system_->getDOF() << std::endl;
  const auto bodyNames = system_->getBodyNames();
  std::cerr << "Body Names are";
  for (const auto& bodyName : bodyNames) {
    std::cerr << "\t" << bodyName;
  }
  std::cerr << std::endl;
  const auto movableJointNames = system_->getMovableJointNames();
  std::cerr << "Movable Joint Names are";
  for (const auto& movableJointName : movableJointNames) {
    std::cerr << "\t" << movableJointName;
  }
  std::cerr << std::endl;

  ground_ = world_.addGround();

  if (raisimRolloutSettings_.raisimServer_) {
    serverPtr_.reset(new raisim::RaisimServer(&world_));
    serverPtr_->launchServer(raisimRolloutSettings_.portNumber_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RaisimRollout::RaisimRollout(const RaisimRollout& other)
    : RaisimRollout(other.urdfFile_, other.resourcePath_, other.stateToRaisimGenCoordGenVel_, other.raisimGenCoordGenVelToState_,
                    other.inputToRaisimGeneralizedForce_, other.dataExtractionCallback_, other.raisimRolloutSettings_,
                    other.inputToRaisimPdTargets_) {
  if (other.heightMap_ != nullptr) {
    deleteGroundPlane();
    heightMap_ = world_.addHeightMap(other.heightMap_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RaisimRollout::~RaisimRollout() {
  if (raisimRolloutSettings_.raisimServer_) {
    serverPtr_->killServer();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RaisimRollout::setPdGains(const Eigen::VectorXd& pGain, const Eigen::VectorXd& dGain) {
  raisimRolloutSettings_.pGains_ = pGain;
  raisimRolloutSettings_.dGains_ = dGain;
  system_->setPdGains(pGain, dGain);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t RaisimRollout::run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, ControllerBase* controller,
                            ModeSchedule& modeSchedule, scalar_array_t& timeTrajectory, size_array_t& postEventIndices,
                            vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  if (initTime > finalTime) {
    throw std::runtime_error("[RaisimRollout::run] The initial time should be less-equal to the final time!");
  }
  if (controller == nullptr) {
    throw std::runtime_error("[RaisimRollout::run] Controller is not set!");
  }

  world_.setTimeStep(this->settings().timeStep);

  // extract sub-systems
  const auto timeIntervalArray = findActiveModesTimeInterval(initTime, finalTime, modeSchedule.eventTimes);
  const int numSubsystems = timeIntervalArray.size();
  const auto maxNumSteps = static_cast<int>(std::round((finalTime - initTime) / this->settings().timeStep));

  // Prepare arrays
  timeTrajectory.clear();
  timeTrajectory.reserve(maxNumSteps + numSubsystems);
  stateTrajectory.clear();
  stateTrajectory.reserve(maxNumSteps + numSubsystems);
  inputTrajectory.clear();
  inputTrajectory.reserve(maxNumSteps + numSubsystems);
  postEventIndices.clear();
  postEventIndices.reserve(numSubsystems - 1);

  // Set inital state to simulation if requested
  if (raisimRolloutSettings_.setSimulatorStateOnRolloutRunAlways_ or raisimRolloutSettings_.setSimulatorStateOnRolloutRunOnce_) {
    Eigen::VectorXd q_init, dq_init;
    inputTrajectory.emplace_back(controller->computeInput(timeIntervalArray.front().first, initState));
    std::tie(q_init, dq_init) = stateToRaisimGenCoordGenVel_(initState, inputTrajectory.back());
    assert(system_->getGeneralizedCoordinateDim() == q_init.rows());
    system_->setState(q_init, dq_init);
    raisimRolloutSettings_.setSimulatorStateOnRolloutRunOnce_ = false;
  }

  // loop through intervals and integrate each separately
  for (const auto& interval : timeIntervalArray) {
    runSimulation(interval, controller, timeTrajectory, stateTrajectory, inputTrajectory);
    postEventIndices.push_back(stateTrajectory.size());
  }
  postEventIndices.pop_back();  // the last interval does not have any events afterwards

  return stateTrajectory.back();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
raisim::HeightMap* RaisimRollout::generateTerrain(raisim::TerrainProperties properties) {
  deleteGroundPlane();

  heightMap_ = world_.addHeightMap(0.0, 0.0, properties);
  return heightMap_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RaisimRollout::setTerrain(const raisim::HeightMap& heightMap) {
  deleteGroundPlane();
  heightMap_ = world_.addHeightMap(&heightMap);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RaisimRollout::setTerrain(const std::string& pngFileName, double centerX, double centerY, double xSize, double ySize,
                               double heightScale, double heightOffset) {
  deleteGroundPlane();
  heightMap_ = world_.addHeightMap(pngFileName, centerX, centerY, xSize, ySize, heightScale, heightOffset);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const raisim::HeightMap* RaisimRollout::getTerrain() const {
  return heightMap_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RaisimRollout::deleteGroundPlane() {
  if (ground_ != nullptr) {
    world_.removeObject(ground_);
    ground_ = nullptr;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RaisimRollout::runSimulation(const std::pair<scalar_t, scalar_t>& timeInterval, ControllerBase* controller,
                                  scalar_array_t& timeTrajectory, vector_array_t& stateTrajectory, vector_array_t& inputTrajectory) {
  const auto numSteps = static_cast<int>(std::ceil((timeInterval.second - timeInterval.first) / this->settings().timeStep));

  for (int i = 0; i < numSteps; i++) {
    const auto time = timeInterval.first + i * this->settings().timeStep;

    if (i == (numSteps - 1)) {
      // last step uses potentially smaller time step
      scalar_t shortened_dt = timeInterval.second - time;
      if (shortened_dt < 1e-4) {
        break;  // for numerical reasons, don't make a tiny step at the end
      }
      world_.setTimeStep(shortened_dt);
    } else {
      world_.setTimeStep(this->settings().timeStep);
    }

    world_.integrate1();  // prepares all kinematic and dynamic quantities for the current time step

    if (i % raisimRolloutSettings_.controlDecimation_ == 0) {
      timeTrajectory.push_back(time);

      if (dataExtractionCallback_) {
        dataExtractionCallback_(time, *system_);  // must run before evaluating controller
      }

      Eigen::VectorXd raisim_q, raisim_dq;
      system_->getState(raisim_q, raisim_dq);
      stateTrajectory.emplace_back(raisimGenCoordGenVelToState_(raisim_q, raisim_dq));
      if (stateTrajectory.back().hasNaN()) {
        throw std::runtime_error("[RaisimRollout::runSimulation] nan in state");
      }

      // input might have been computed by initialization already
      if (inputTrajectory.size() < stateTrajectory.size()) {
        inputTrajectory.emplace_back(controller->computeInput(time, stateTrajectory.back()));
      }
      Eigen::VectorXd tau = inputToRaisimGeneralizedForce_(time, inputTrajectory.back(), stateTrajectory.back(), raisim_q, raisim_dq);
      assert(tau.rows() == system_->getDOF());
      system_->setGeneralizedForce(tau);

      if (system_->getControlMode() != raisim::ControlMode::FORCE_AND_TORQUE) {
        Eigen::VectorXd pGain, dGain;
        std::tie(pGain, dGain) = inputToRaisimPdTargets_(time, inputTrajectory.back(), stateTrajectory.back(), raisim_q, raisim_dq);
        system_->setPdTarget(pGain, dGain);
      }
    }

    world_.integrate2();  // actually move time foward and change the state
  }

  world_.setTimeStep(this->settings().timeStep);

  // also push back final state and input
  timeTrajectory.push_back(timeInterval.second);

  world_.integrate1();

  Eigen::VectorXd raisim_q, raisim_dq;
  system_->getState(raisim_q, raisim_dq);
  stateTrajectory.emplace_back(raisimGenCoordGenVelToState_(raisim_q, raisim_dq));

  if (dataExtractionCallback_) {
    dataExtractionCallback_(timeTrajectory.back(), *system_);
  }

  vector_t input = controller->computeInput(timeTrajectory.back(), stateTrajectory.back());
  inputTrajectory.push_back(input);
}

}  // namespace ocs2
