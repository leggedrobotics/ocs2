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

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/cost/CostCollection.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_mobile_manipulator_example/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator_example/constraint/EndEffectorConstraint.h>

namespace ocs2 {
namespace mobile_manipulator {

class MobileManipulatorCost : public CostFunctionBase {
 public:
  using quaternion_t = Eigen::Quaternion<scalar_t>;

  MobileManipulatorCost(PinocchioInterface pinocchioInterface, const std::string& taskFile, bool useCaching,
                        const std::string& libraryFolder, bool recompileLibraries);
  ~MobileManipulatorCost() override = default;
  MobileManipulatorCost* clone() const override { return new MobileManipulatorCost(*this); }

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  scalar_t finalCost(scalar_t t, const vector_t& x) override;
  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override;

 private:
  MobileManipulatorCost(const MobileManipulatorCost& rhs);

  void setCachePointers();
  void setEndEffectorReference(scalar_t time);
  void setFinalEndEffectorReference(scalar_t time);
  std::pair<vector_t, quaternion_t> interpolateEndEffectorPose(const TargetTrajectories& targetTrajectories, scalar_t time) const;

  std::unique_ptr<StateInputCost> getQuadraticInputCost(const std::string& taskFile);
  std::unique_ptr<StateCost> getEndEffectorCost(const std::string& taskFile, const std::string& fieldName, const std::string& libraryFolder,
                                                bool recompileLibraries);
  std::unique_ptr<StateCost> getSelfCollisionCost(const std::string& taskFile, const std::string& libraryFolder, bool recompileLibraries);
  std::unique_ptr<StateInputCost> getJointVelocityLimitCost(const std::string& taskFile);

  bool useCaching_;

  PinocchioInterface pinocchioInterface_;
  CostCollection<StateInputCost> stateInputCostCollection_;
  CostCollection<StateCost> stateCostCollection_;
  CostCollection<StateCost> finalCostCollection_;

  MobileManipulatorPinocchioMapping<scalar_t> pinocchioMapping_;

  EndEffectorConstraint* eeConstraintPtr_ = nullptr;
  EndEffectorConstraint* finalEeConstraintPtr_ = nullptr;
};

}  // namespace mobile_manipulator
}  // namespace ocs2
