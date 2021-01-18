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
#include <ocs2_mobile_manipulator_example/cost/EndEffectorCost.h>
#include <ocs2_mobile_manipulator_example/cost/MobileManipulatorCost.h>

//  #include <ocs2_self_collision/cost/SelfCollisionCost.h>
//  #include <ocs2_self_collision/cost/SelfCollisionCostCppAd.h>
#include <ocs2_self_collision/loadStdVectorOfPair.h>

#include <ocs2_core/soft_constraint/penalties/QuadraticPenaltyFunction.h>
#include <ocs2_core/soft_constraint/penalties/SmoothAbsolutePenaltyFunction.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <ros/package.h>

namespace mobile_manipulator {

class QuadraticInputCost : public ocs2::CostFunctionBase {
 public:
  QuadraticInputCost(matrix_t R) : R_(std::move(R)) {}
  ~QuadraticInputCost() override = default;
  QuadraticInputCost* clone() const override { return new QuadraticInputCost(*this); }

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override {
    if (costDesiredTrajectoriesPtr_ == nullptr) {
      throw std::runtime_error("[QuadraticInputCost] costDesiredTrajectoriesPtr_ is not set. Use setCostDesiredTrajectoriesPtr()");
    }
    const vector_t uDeviation = u - costDesiredTrajectoriesPtr_->getDesiredInput(t);
    return 0.5 * uDeviation.dot(R_ * uDeviation);
  }
  scalar_t finalCost(scalar_t t, const vector_t& x) override { return 0.0; }
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override {
    if (costDesiredTrajectoriesPtr_ == nullptr) {
      throw std::runtime_error("[QuadraticInputCost] costDesiredTrajectoriesPtr_ is not set. Use setCostDesiredTrajectoriesPtr()");
    }
    const vector_t uDeviation = u - costDesiredTrajectoriesPtr_->getDesiredInput(t);
    const vector_t rDeviation = R_ * uDeviation;

    ScalarFunctionQuadraticApproximation L = ScalarFunctionQuadraticApproximation::Zero(x.rows(), u.rows());
    L.f = 0.5 * uDeviation.dot(rDeviation);
    L.dfdu = rDeviation;
    L.dfduu = R_;
    return L;
  }
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override {
    return ScalarFunctionQuadraticApproximation::Zero(x.rows(), 0);
  }

 private:
  matrix_t R_;
};

std::unique_ptr<MobileManipulatorCost> getMobileManipulatorCost(const ocs2::PinocchioInterface& pinocchioInterface,
                                                                const std::string& taskFile, const std::string& libraryFolder,
                                                                bool recompileLibraries) {
  using WeightedCost = ocs2::CostFunctionLinearCombination::WeightedCost;
  const bool verbose = true;

  std::vector<WeightedCost> costs;

  /* End effector tracking cost */
  ocs2::QuadraticPenaltyFunction penalty(1.0);
  // ocs2::SmoothAbsolutePenaltyFunction penalty(ocs2::SmoothAbsolutePenaltyFunction::Config(1.0, 1e-2));
  MobileManipulatorPinocchioMapping<scalar_t> pinocchioMapping;
  const std::vector<std::string> eeNames = {"WRIST_2"};
  ocs2::PinocchioEndEffectorKinematics eeKinematics(pinocchioInterface, pinocchioMapping, eeNames);
  auto eeCostPtr = std::make_shared<EndEffectorCost>(eeKinematics, penalty);

  scalar_t eeCostWeight = 1.0;
  ocs2::loadData::loadCppDataType(taskFile, "endEffectorCost.weight", eeCostWeight);
  std::cerr << "EndEffectorCost weight: " << eeCostWeight << std::endl;

  const size_t maxNumPairs = 100;
  // TODO(perry) replace with some nice link parser or something
  std::vector<std::pair<size_t, size_t>> selfCollisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> selfCollisionLinkPairs;
  double selfColWeight, mu, delta, minimumDistance = 1.0;

  ocs2::loadData::loadStdVectorOfPair(taskFile, "selfCollisionCost.collisionObjectPairs", selfCollisionObjectPairs);
  ocs2::loadData::loadStdVectorOfPair(taskFile, "selfCollisionCost.collisionLinkPairs", selfCollisionLinkPairs);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollisionCost.weight", selfColWeight);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollisionCost.mu", mu);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollisionCost.delta", delta);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollisionCost.minimumDistance", minimumDistance);
  std::cerr << "selfColWeight:  " << selfColWeight << std::endl;
  std::cerr << "mu:  " << mu << std::endl;
  std::cerr << "delta:  " << delta << std::endl;
  std::cerr << "minimumDistance:  " << minimumDistance << std::endl;
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

  // ocs2::PinocchioGeometryInterface geometryInterface(urdfPath_, pinocchioInterface, selfCollisionLinkPairs, selfCollisionObjectPairs);

  // auto colCost = std::make_shared<ocs2::SelfCollisionCost>(pinocchioInterface, geometryInterface, minimumDistance, mu, delta);
  // auto colCostAd = std::make_shared<ocs2::SelfCollisionCostCppAd>(pinocchioInterface, geometryInterface, minimumDistance, mu, delta);
  // colCostAd->initialize("ColCostAd", libraryFolder, recompileLibraries, verbose);

  auto inputCost = std::make_shared<QuadraticInputCost>(matrix_t::Identity(INPUT_DIM, INPUT_DIM));
  costs.emplace_back(WeightedCost{0.1, std::move(inputCost)});
  costs.emplace_back(WeightedCost{eeCostWeight, std::move(eeCostPtr)});
  //  costs.emplace_back(WeightedCost{selfColWeight, std::move(colCost)});
  //  costs.emplace_back(WeightedCost{selfColWeight, std::move(colCostAd)});

  // TODO(mspieler): use make_unique after switch to C++14
  return std::unique_ptr<MobileManipulatorCost>(new MobileManipulatorCost(std::move(costs)));
}

}  // namespace mobile_manipulator
