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

#include <ocs2_mobile_manipulator_example/cost/EndEffectorCost.h>
#include <ocs2_mobile_manipulator_example/cost/MobileManipulatorCost.h>
#include <ocs2_mobile_manipulator_example/cost/SelfCollisionCost.h>

#include <ros/package.h>
namespace mobile_manipulator {

std::unique_ptr<MobileManipulatorCost> getMobileManipulatorCost(const PinocchioInterface<ad_scalar_t>& pinocchioInterface,
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

  std::string urdfPath_ = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";

  PinocchioInterface<scalar_t> pinocchioInterfaceDouble(urdfPath_);
  PinocchioGeometryInterface geometryInterface(urdfPath_, pinocchioInterfaceDouble, {{1, 6}});

  auto colCost = std::make_shared<SelfCollisionCost>(pinocchioInterfaceDouble, geometryInterface, 0.1, 0.01, 1e-3);

  costs.emplace_back(WeightedCost{eeCostWeight, std::move(eeCostPtr)});
  costs.emplace_back(WeightedCost{0.01, std::move(colCost)});

  // TODO(mspieler): use make_unique after switch to C++14
  return std::unique_ptr<MobileManipulatorCost>(new MobileManipulatorCost(std::move(costs)));
}

}  // namespace mobile_manipulator
