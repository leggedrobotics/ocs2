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
#include <ocs2_mobile_manipulator_example/cost/EndEffectorCost.h>
#include <ocs2_mobile_manipulator_example/cost/MobileManipulatorCost.h>
#include <ocs2_pinocchio/ExtendedPair.h>
#include <ocs2_pinocchio/cost/SelfCollisionCost.h>
#include <ocs2_pinocchio/cost/SelfCollisionCostCppAd.h>

#include <ros/package.h>
namespace mobile_manipulator {

std::unique_ptr<MobileManipulatorCost> getMobileManipulatorCost(const ocs2::PinocchioInterface<ad_scalar_t>& pinocchioInterface,
                                                                const std::string& taskFile, const std::string& libraryFolder,
                                                                bool recompileLibraries) {
  using WeightedCost = ocs2::CostFunctionLinearCombination::WeightedCost;
  const bool verbose = true;

  std::vector<WeightedCost> costs;

  /* End effector tracking cost */
  matrix_t Q(6, 6), R(INPUT_DIM, INPUT_DIM), Qf(6, 6);
  ocs2::loadData::loadEigenMatrix(taskFile, "endEffectorCost.Q", Q);
  ocs2::loadData::loadEigenMatrix(taskFile, "endEffectorCost.R", R);
  ocs2::loadData::loadEigenMatrix(taskFile, "endEffectorCost.Q_final", Qf);
  std::cerr << "Q:  \n" << Q << std::endl;
  std::cerr << "R:  \n" << R << std::endl;
  std::cerr << "Q_final:\n" << Qf << std::endl;
  ocs2::CostDesiredTrajectories initCostDesiredTrajectory({0.0}, {vector_t::Zero(STATE_DIM)}, {vector_t::Zero(INPUT_DIM)});

  scalar_t eeCostWeight = 1.0;
  ocs2::loadData::loadCppDataType(taskFile, "endEffectorCost.weight", eeCostWeight);
  std::cerr << "EndEffectorCost weight: " << eeCostWeight << std::endl;

  auto eeCostPtr = std::make_shared<EndEffectorCost>(pinocchioInterface, std::move(Q), std::move(R), std::move(Qf));
  eeCostPtr->setCostDesiredTrajectoriesPtr(&initCostDesiredTrajectory);  // required for CppAD initialization pass
  eeCostPtr->initialize("EndEffectorCost", libraryFolder, recompileLibraries, verbose);

  const size_t maxNumPairs = 100;
  // TODO(perry) replace with some nice link parser or something
  std::vector<ocs2::ExtendedPair<size_t, size_t>> selfCollisionPairs;
  scalar_t selfColWeight, mu, delta, minimumDistance = 1.0;

  ocs2::loadData::loadStdVector(taskFile, "selfCollisionCost.collisionPairs", selfCollisionPairs);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollisionCost.weight", selfColWeight);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollisionCost.mu", mu);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollisionCost.delta", delta);
  ocs2::loadData::loadCppDataType(taskFile, "selfCollisionCost.minimumDistance", minimumDistance);
  std::cerr << "selfColWeight:  " << selfColWeight << std::endl;
  std::cerr << "mu:  " << mu << std::endl;
  std::cerr << "delta:  " << delta << std::endl;
  std::cerr << "minimumDistance:  " << minimumDistance << std::endl;

  std::string urdfPath_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";
  ocs2::PinocchioInterface<scalar_t> pinocchioInterfaceScalar = MobileManipulatorInterface::buildPinocchioInterface(urdfPath_);
  ocs2::PinocchioGeometryInterface geometryInterface(urdfPath_, pinocchioInterfaceScalar, selfCollisionPairs);
  auto colCost = std::make_shared<ocs2::SelfCollisionCost>(pinocchioInterfaceScalar, geometryInterface, minimumDistance, mu, delta);
  //  auto colCostAd = std::make_shared<ocs2::SelfCollisionCostCppAd>(pinocchioInterfaceScalar, geometryInterface, minimumDistance, mu,
  //  delta); colCostAd->initialize("ColCostAd", libraryFolder, recompileLibraries, verbose);

  costs.emplace_back(WeightedCost{eeCostWeight, std::move(eeCostPtr)});
  costs.emplace_back(WeightedCost{selfColWeight, std::move(colCost)});
  //  costs.emplace_back(WeightedCost{selfColWeight, std::move(colCostAd)});

  // TODO(mspieler): use make_unique after switch to C++14
  return std::unique_ptr<MobileManipulatorCost>(new MobileManipulatorCost(std::move(costs)));
}

}  // namespace mobile_manipulator
