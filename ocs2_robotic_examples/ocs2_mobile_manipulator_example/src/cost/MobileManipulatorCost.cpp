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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_mobile_manipulator_example/constraint/SelfCollisionConstraint.h>
// #include <ocs2_mobile_manipulator_example/constraint/SelfCollisionConstraintCppAd.h>
#include <ocs2_mobile_manipulator_example/cost/MobileManipulatorCost.h>
#include <ocs2_mobile_manipulator_example/cost/QuadraticInputCost.h>
#include <ocs2_mobile_manipulator_example/definitions.h>

#include <ocs2_self_collision/loadStdVectorOfPair.h>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/soft_constraint/penalties/QuadraticPenaltyFunction.h>
#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenaltyFunction.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <ros/package.h>

namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorCost::MobileManipulatorCost(ocs2::PinocchioInterface pinocchioInterface, const std::string& taskFile,
                                             const std::string& libraryFolder, bool recompileLibraries)
    : pinocchioInterface_(std::move(pinocchioInterface)) {
  auto inputCost = getQuadraticInputCost(taskFile);
  stateInputCostCollection_.add("Input", std::move(inputCost));

  auto selfCollisionCost = getSelfCollisionCost(taskFile, libraryFolder, recompileLibraries);
  stateCostCollection_.add("SelfCollision", std::move(selfCollisionCost));

  auto eeCost = getEndEffectorCost(taskFile, "endEffector", libraryFolder, recompileLibraries);
  stateCostCollection_.add("EndEffector", std::move(eeCost));

  auto finalEeCost = getEndEffectorCost(taskFile, "finalEndEffector", libraryFolder, recompileLibraries);
  finalCostCollection_.add("FinalEndEffector", std::move(finalEeCost));

  setCachePointers();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorCost::MobileManipulatorCost(const MobileManipulatorCost& rhs)
    : pinocchioInterface_(rhs.pinocchioInterface_),
      stateInputCostCollection_(rhs.stateInputCostCollection_),
      stateCostCollection_(rhs.stateCostCollection_),
      finalCostCollection_(rhs.finalCostCollection_) {
  setCachePointers();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorCost::setCachePointers() {
  auto* selfCollisionConstraintPtr =
      dynamic_cast<SelfCollisionConstraint*>(&stateCostCollection_.get<ocs2::StateSoftConstraint>("SelfCollision").get());
  if (selfCollisionConstraintPtr != nullptr) {
    selfCollisionConstraintPtr->setPinocchioInterface(pinocchioInterface_);
  }

  eeConstraintPtr_ = &stateCostCollection_.get<ocs2::StateSoftConstraint>("EndEffector").get<EndEffectorConstraint>();
  auto* kinematicsPtr = dynamic_cast<ocs2::PinocchioEndEffectorKinematics*>(&eeConstraintPtr_->getEndEffectorKinematics());
  if (kinematicsPtr != nullptr) {
    kinematicsPtr->setPinocchioInterface(pinocchioInterface_);
  }

  finalEeConstraintPtr_ = &finalCostCollection_.get<ocs2::StateSoftConstraint>("FinalEndEffector").get<EndEffectorConstraint>();
  kinematicsPtr = dynamic_cast<ocs2::PinocchioEndEffectorKinematics*>(&finalEeConstraintPtr_->getEndEffectorKinematics());
  if (kinematicsPtr != nullptr) {
    kinematicsPtr->setPinocchioInterface(pinocchioInterface_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MobileManipulatorCost::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  setEndEffectorReference(t);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  scalar_t cost = stateInputCostCollection_.getValue(t, x, u, *costDesiredTrajectoriesPtr_);
  cost += stateCostCollection_.getValue(t, x, *costDesiredTrajectoriesPtr_);
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MobileManipulatorCost::finalCost(scalar_t t, const vector_t& x) {
  setFinalEndEffectorReference(t);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  return finalCostCollection_.getValue(t, x, *costDesiredTrajectoriesPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MobileManipulatorCost::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  setEndEffectorReference(t);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateGlobalPlacements(model, data);

  auto cost = stateInputCostCollection_.getQuadraticApproximation(t, x, u, *costDesiredTrajectoriesPtr_);
  const auto stateCost = stateCostCollection_.getQuadraticApproximation(t, x, *costDesiredTrajectoriesPtr_);
  cost.f += stateCost.f;
  cost.dfdx += stateCost.dfdx;
  cost.dfdxx += stateCost.dfdxx;
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MobileManipulatorCost::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  setFinalEndEffectorReference(t);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(x);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  return finalCostCollection_.getQuadraticApproximation(t, x, *costDesiredTrajectoriesPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorCost::setEndEffectorReference(scalar_t time) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[MobileManipulatorCost] costDesiredTrajectoriesPtr_ is not set.");
  }

  ocs2::vector_t eePosition;
  quaternion_t eeOrientation;
  std::tie(eePosition, eeOrientation) = interpolateEndEffectorPose(*costDesiredTrajectoriesPtr_, time);

  eeConstraintPtr_->setDesiredPose(eePosition, eeOrientation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorCost::setFinalEndEffectorReference(scalar_t time) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[MobileManipulatorCost] costDesiredTrajectoriesPtr_ is not set.");
  }

  ocs2::vector_t eePosition;
  quaternion_t eeOrientation;
  std::tie(eePosition, eeOrientation) = interpolateEndEffectorPose(*costDesiredTrajectoriesPtr_, time);

  finalEeConstraintPtr_->setDesiredPose(eePosition, eeOrientation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto MobileManipulatorCost::interpolateEndEffectorPose(const ocs2::CostDesiredTrajectories& costDesiredTrajectory, scalar_t time) const
    -> std::pair<vector_t, quaternion_t> {
  const auto& desiredTrajectory = costDesiredTrajectory.desiredStateTrajectory();
  std::pair<vector_t, Eigen::Quaternion<scalar_t>> reference;

  if (desiredTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = ocs2::LinearInterpolation::timeSegment(time, costDesiredTrajectory.desiredTimeTrajectory());

    const auto& lhs = desiredTrajectory[index];
    const auto& rhs = desiredTrajectory[index + 1];
    const quaternion_t quaternionA(lhs.tail<4>());
    const quaternion_t quaternionB(rhs.tail<4>());

    reference.first = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
    reference.second = quaternionA.slerp((1 - alpha), quaternionB);
  } else {  // desiredTrajectory.size() == 1
    reference.first = desiredTrajectory[0].head<3>();
    reference.second = quaternion_t(desiredTrajectory[0].tail<4>());
  }

  return reference;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::StateInputCost> MobileManipulatorCost::getQuadraticInputCost(const std::string& taskFile) {
  matrix_t R(INPUT_DIM, INPUT_DIM);
  ocs2::loadData::loadEigenMatrix(taskFile, "inputCost.R", R);
  std::cerr << "inputCost.R:  \n" << R << std::endl;

  return std::unique_ptr<ocs2::StateInputCost>(new QuadraticInputCost(std::move(R)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::StateCost> MobileManipulatorCost::getEndEffectorCost(const std::string& taskFile, const std::string& fieldName,
                                                                           const std::string& libraryFolder, bool recompileLibraries) {
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  std::string name = "WRIST_2";
  bool useCppAd = false;

  ocs2::loadData::loadCppDataType(taskFile, fieldName + ".muPosition", muPosition);
  ocs2::loadData::loadCppDataType(taskFile, fieldName + ".muOrientation", muOrientation);
  ocs2::loadData::loadCppDataType(taskFile, fieldName + ".name", name);
  ocs2::loadData::loadCppDataType(taskFile, fieldName + ".useCppAd", useCppAd);

  std::cerr << fieldName << ".muPosition:  " << muPosition << std::endl;
  std::cerr << fieldName << ".muOrientation:  " << muOrientation << std::endl;
  std::cerr << fieldName << ".name:  " << name << std::endl;
  std::cerr << fieldName << ".useCppAd:  " << useCppAd << std::endl;

  std::unique_ptr<ocs2::StateConstraint> eeConstraint;
  if (!useCppAd) {
    ocs2::PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface_, pinocchioMapping_, {name});
    eeConstraint = std::unique_ptr<ocs2::StateConstraint>(new EndEffectorConstraint(eeKinematics));
  } else {
    MobileManipulatorPinocchioMapping<ocs2::ad_scalar_t> pinocchioMappingCppAd;
    ocs2::PinocchioEndEffectorKinematicsCppAd eeKinematics(pinocchioInterface_, pinocchioMappingCppAd, {name});
    eeKinematics.initialize(STATE_DIM, INPUT_DIM, "end_effector_kinematics", libraryFolder, recompileLibraries, false);
    eeConstraint = std::unique_ptr<ocs2::StateConstraint>(new EndEffectorConstraint(eeKinematics));
  }

  std::vector<std::unique_ptr<ocs2::PenaltyFunctionBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3,
                  [&] { return std::unique_ptr<ocs2::PenaltyFunctionBase>(new ocs2::QuadraticPenaltyFunction(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3,
                  [&] { return std::unique_ptr<ocs2::PenaltyFunctionBase>(new ocs2::QuadraticPenaltyFunction(muOrientation)); });

  return std::unique_ptr<ocs2::StateCost>(
      new ocs2::StateSoftConstraint(std::move(eeConstraint), std::move(penaltyArray), ocs2::ConstraintOrder::Linear));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::StateCost> MobileManipulatorCost::getSelfCollisionCost(const std::string& taskFile, const std::string& libraryFolder,
                                                                             bool recompileLibraries) {
  bool useCppAd = false;
  ocs2::loadData::loadCppDataType(taskFile, "selfCollision.useCppAd", useCppAd);
  if (useCppAd) {
    throw std::runtime_error("[MobileManipulatorCost] SelfCollisionConstraintCppAd not implemented");
  }

  // TODO(perry) replace with some nice link parser or something
  std::vector<std::pair<size_t, size_t>> selfCollisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> selfCollisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 1.0;

  ocs2::loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs", selfCollisionObjectPairs);
  ocs2::loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs", selfCollisionLinkPairs);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollision.mu", mu);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollision.delta", delta);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollision.minimumDistance", minimumDistance);

  std::cerr << "selfCollision.mu:  " << mu << std::endl;
  std::cerr << "selfCollision.delta:  " << delta << std::endl;
  std::cerr << "selfCollision.minimumDistance:  " << minimumDistance << std::endl;
  std::cout << "Loaded collision object pairs: ";
  for (const auto& element : selfCollisionObjectPairs) {
    std::cout << "[" << element.first << ", " << element.second << "]; ";
  }
  std::cout << std::endl;
  std::cout << "Loaded collision link pairs: ";
  for (const auto& element : selfCollisionLinkPairs) {
    std::cout << "[" << element.first << ", " << element.second << "]; ";
  }
  std::cout << std::endl;

  std::string urdfPath_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";

  ocs2::PinocchioGeometryInterface geometryInterface(urdfPath_, pinocchioInterface_, selfCollisionLinkPairs, selfCollisionObjectPairs);

  // Note: selfCollisionLinkPairs and selfCollisionObjectPairs might contain invalid pairs.
  const size_t numCollisionPairs = geometryInterface.getNumCollisionPairs();

  auto constraint = std::unique_ptr<ocs2::StateConstraint>(
      new SelfCollisionConstraint(MobileManipulatorPinocchioMapping<scalar_t>(), std::move(geometryInterface), minimumDistance));

  auto penalty = std::unique_ptr<ocs2::PenaltyFunctionBase>(
      new ocs2::RelaxedBarrierPenaltyFunction(ocs2::RelaxedBarrierPenaltyFunction::Config(mu, delta)));

  return std::unique_ptr<ocs2::StateCost>(
      new ocs2::StateSoftConstraint(std::move(constraint), numCollisionPairs, std::move(penalty), ocs2::ConstraintOrder::Linear));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<MobileManipulatorCost> getMobileManipulatorCost(const ocs2::PinocchioInterface& pinocchioInterface,
                                                                const std::string& taskFile, const std::string& libraryFolder,
                                                                bool recompileLibraries) {
  // TODO(mspieler): use make_unique after switch to C++14
  return std::unique_ptr<MobileManipulatorCost>(new MobileManipulatorCost(pinocchioInterface, taskFile, libraryFolder, recompileLibraries));
}

}  // namespace mobile_manipulator
