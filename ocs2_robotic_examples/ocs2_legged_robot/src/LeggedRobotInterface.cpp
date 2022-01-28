/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_legged_robot/LeggedRobotInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "ocs2_legged_robot/LeggedRobotPreComputation.h"
#include "ocs2_legged_robot/constraint/FrictionConeConstraint.h"
#include "ocs2_legged_robot/constraint/NormalVelocityConstraintCppAd.h"
#include "ocs2_legged_robot/constraint/ZeroForceConstraint.h"
#include "ocs2_legged_robot/constraint/ZeroVelocityConstraintCppAd.h"
#include "ocs2_legged_robot/cost/LeggedRobotStateInputQuadraticCost.h"
#include "ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInterface::LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  bool verbose;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
  sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting", verbose);

  // OptimalConrolProblem
  setupOptimalConrolProblem(taskFile, urdfFile, referenceFile, verbose);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::setupOptimalConrolProblem(const std::string& taskFile, const std::string& urdfFile,
                                                     const std::string& referenceFile, bool verbose) {
  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
      modelSettings_.contactNames6DoF);

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4));

  // Mode schedule manager
  referenceManagerPtr_ =
      std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner));

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  bool useAnalyticalGradientsDynamics = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  if (useAnalyticalGradientsDynamics) {
    throw std::runtime_error("[LeggedRobotInterface::setupOptimalConrolProblem] The analytical dynamics class is not yet implemented!");
  } else {
    const std::string modelName = "dynamics";
    dynamicsPtr.reset(new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));
  }

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, false));

  // Constraint terms
  // friction cone settings
  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);

  bool useAnalyticalGradientsConstraints = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;
    if (useAnalyticalGradientsConstraints) {
      throw std::runtime_error(
          "[LeggedRobotInterface::setupOptimalConrolProblem] The analytical end-effector linear constraint is not implemented!");
    } else {
      const auto infoCppAd = centroidalModelInfo_.toCppAd();
      const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
      auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
        const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
        updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
      };
      eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                                                                    centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                    velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                                                                    modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
    }

    problemPtr_->softConstraintPtr->add(footName + "_frictionCone",
                                        getFrictionConeConstraint(i, frictionCoefficient, barrierPenaltyConfig));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", getZeroForceConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity",
                                            getZeroVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity",
                                            getNormalVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                     *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(new LeggedRobotInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedRobotInterface::loadGaitSchedule(const std::string& file, bool verbose) const {
  const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  if (verbose) {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LeggedRobotInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info) {
  const size_t totalContactDim = 3 * info.numThreeDofContacts;

  vector_t initialState(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState);

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_t baseToFeetJacobians(totalContactDim, info.actuatedDofNum);
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                jacobianWorldToContactPointInWorldFrame);

    baseToFeetJacobians.block(3 * i, 0, 3, info.actuatedDofNum) =
        jacobianWorldToContactPointInWorldFrame.block(0, 6, 3, info.actuatedDofNum);
  }

  matrix_t R_taskspace(totalContactDim + totalContactDim, totalContactDim + totalContactDim);
  loadData::loadEigenMatrix(taskFile, "R", R_taskspace);

  matrix_t R = matrix_t::Zero(info.inputDim, info.inputDim);
  // Contact Forces
  R.topLeftCorner(totalContactDim, totalContactDim) = R_taskspace.topLeftCorner(totalContactDim, totalContactDim);
  // Joint velocities
  R.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
      baseToFeetJacobians.transpose() * R_taskspace.bottomRightCorner(totalContactDim, totalContactDim) * baseToFeetJacobians;
  return R;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info,
                                                                          bool verbose) {
  matrix_t Q(info.stateDim, info.stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R = initializeInputCostWeight(taskFile, info);

  if (verbose) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(new LeggedRobotStateInputQuadraticCost(std::move(Q), std::move(R), info, *referenceManagerPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedRobotInterface::loadFrictionConeSettings(const std::string& taskFile,
                                                                                                  bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";

  scalar_t frictionCoefficient = 1.0;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return {frictionCoefficient, std::move(barrierPenaltyConfig)};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  std::unique_ptr<FrictionConeConstraint> frictionConeConstraintPtr(
      new FrictionConeConstraint(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex, centroidalModelInfo_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(frictionConeConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroForceConstraint(size_t contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      size_t contactPointIndex,
                                                                                      bool useAnalyticalGradients) {
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax.setZero(3, 3);
      config.Ax(2, 2) = positionErrorGain;
    }
    return config;
  };

  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::getZeroVelocityConstraint] The analytical end-effector zero velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                                                                 eeZeroVelConConfig(modelSettings_.positionErrorGain)));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                        size_t contactPointIndex,
                                                                                        bool useAnalyticalGradients) {
  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::getNormalVelocityConstraint] The analytical end-effector normal velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
  }
}

}  // namespace legged_robot
}  // namespace ocs2
