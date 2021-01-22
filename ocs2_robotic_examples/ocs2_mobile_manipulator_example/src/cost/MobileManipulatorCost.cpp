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

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_example/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator_example/cost/MobileManipulatorCost.h>
#include <ocs2_mobile_manipulator_example/cost/QuadraticInputCost.h>
#include <ocs2_mobile_manipulator_example/definitions.h>

#include <ocs2_self_collision/loadStdVectorOfPair.h>

#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/soft_constraint/penalties/QuadraticPenaltyFunction.h>
#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenaltyFunction.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
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
  stateInputCosts_.add("Input", std::move(inputCost));

  auto selfCollisionCost = getSelfCollisionCost(taskFile, libraryFolder, recompileLibraries);
  stateCosts_.add("SelfCollision", std::move(selfCollisionCost));

  auto eeCost = getEndEffectorCost(taskFile, "endEffector");
  stateCosts_.add("EndEffector", std::move(eeCost));

  auto finalEeCost = getEndEffectorCost(taskFile, "finalEndEffector");
  finalCosts_.add("FinalEndEffector", std::move(finalEeCost));

  setCachePointers();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MobileManipulatorCost::MobileManipulatorCost(const MobileManipulatorCost& rhs)
    : pinocchioInterface_(rhs.pinocchioInterface_),
      stateInputCosts_(rhs.stateInputCosts_),
      stateCosts_(rhs.stateCosts_),
      finalCosts_(rhs.finalCosts_) {
  setCachePointers();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MobileManipulatorCost::setCachePointers() {
  selfCollisionConstraintPtr_ = &stateCosts_.get<ocs2::StateSoftConstraint>("SelfCollision").get<SelfCollisionConstraint>();
  selfCollisionConstraintPtr_->setPinocchioInterfacePtr(&pinocchioInterface_);

  eeConstraintPtr_ = &stateCosts_.get<ocs2::StateSoftConstraint>("EndEffector").get<EndEffectorConstraint>();

  finalEeConstraintPtr_ = &finalCosts_.get<ocs2::StateSoftConstraint>("FinalEndEffector").get<EndEffectorConstraint>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MobileManipulatorCost::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  setEndEffectorReference(t);
  selfCollisionConstraintPtr_->computeValue(x);

  scalar_t cost = stateInputCosts_.getValue(t, x, u, *costDesiredTrajectoriesPtr_);
  cost += stateCosts_.getValue(t, x, *costDesiredTrajectoriesPtr_);
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t MobileManipulatorCost::finalCost(scalar_t t, const vector_t& x) {
  setFinalEndEffectorReference(t);
  return stateCosts_.getValue(t, x, *costDesiredTrajectoriesPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation MobileManipulatorCost::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  setEndEffectorReference(t);
  selfCollisionConstraintPtr_->computeApproximation(x);

  auto cost = stateInputCosts_.getQuadraticApproximation(t, x, u, *costDesiredTrajectoriesPtr_);
  const auto stateCost = stateCosts_.getQuadraticApproximation(t, x, *costDesiredTrajectoriesPtr_);
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
  return finalCosts_.getQuadraticApproximation(t, x, *costDesiredTrajectoriesPtr_);
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
  std::pair<vector_t, Eigen::Quaternion<scalar_t>> reference;

  const auto& desiredTimeTrajectory = costDesiredTrajectory.desiredTimeTrajectory();
  const auto& desiredStateTrajectory = costDesiredTrajectory.desiredStateTrajectory();

  auto it = std::lower_bound(desiredTimeTrajectory.begin(), desiredTimeTrajectory.end(), time);
  int timeAIdx = it - desiredTimeTrajectory.begin() - 1;
  if (timeAIdx == -1) {
    reference.first = desiredStateTrajectory[0].head<3>();
    reference.second.coeffs() = desiredStateTrajectory[0].tail<4>();
  } else if (timeAIdx == desiredTimeTrajectory.size() - 1) {
    reference.first = desiredStateTrajectory[timeAIdx].head<3>();
    reference.second.coeffs() = desiredStateTrajectory[timeAIdx].tail<4>();
  } else {
    // interpolation
    const scalar_t tau = (time - desiredTimeTrajectory[timeAIdx]) / (desiredTimeTrajectory[timeAIdx + 1] - desiredTimeTrajectory[timeAIdx]);
    const Eigen::Quaternion<scalar_t> quatA(desiredStateTrajectory[timeAIdx].tail<4>());
    const Eigen::Quaternion<scalar_t> quatB(desiredStateTrajectory[timeAIdx + 1].tail<4>());

    reference.first = (1 - tau) * desiredStateTrajectory[timeAIdx].head<3>() + tau * desiredStateTrajectory[timeAIdx + 1].head<3>();
    reference.second = quatA.slerp(tau, quatB);
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
std::unique_ptr<ocs2::StateCost> MobileManipulatorCost::getEndEffectorCost(const std::string& taskFile, const std::string& fieldName) {
  double muPosition = 1.0;
  double muOrientation = 1.0;
  std::string name = "WRIST_2";

  ocs2::loadData::loadCppDataType(taskFile, fieldName + ".muPosition", muPosition);
  ocs2::loadData::loadCppDataType(taskFile, fieldName + ".muOrientation", muOrientation);
  ocs2::loadData::loadCppDataType(taskFile, fieldName + ".name", name);

  std::cerr << fieldName << ".muPosition:  " << muPosition << std::endl;
  std::cerr << fieldName << ".muOrientation:  " << muOrientation << std::endl;
  std::cerr << fieldName << ".name:  " << name << std::endl;

  ocs2::PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface_, MobileManipulatorPinocchioMapping<scalar_t>(), {name});
  auto eeConstraint = std::unique_ptr<ocs2::StateConstraint>(new EndEffectorConstraint(eeKinematics));

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
  double mu = 1e-2;
  double delta = 1e-3;
  double minimumDistance = 1.0;

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
std::unique_ptr<ocs2::CostFunctionBase> getMobileManipulatorCost(const ocs2::PinocchioInterface& pinocchioInterface,
                                                                 const std::string& taskFile, const std::string& libraryFolder,
                                                                 bool recompileLibraries) {
  // TODO(mspieler): use make_unique after switch to C++14
  return std::unique_ptr<MobileManipulatorCost>(new MobileManipulatorCost(pinocchioInterface, taskFile, libraryFolder, recompileLibraries));
}

}  // namespace mobile_manipulator
