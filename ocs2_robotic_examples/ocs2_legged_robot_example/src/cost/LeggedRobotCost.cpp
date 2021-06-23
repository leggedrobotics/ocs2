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

#include <ocs2_legged_robot_example/cost/LeggedRobotCost.h>

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_legged_robot_example/common/definitions.h>
#include <ocs2_legged_robot_example/cost/LeggedRobotStateInputQuadraticCost.h>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <ros/package.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotCost::LeggedRobotCost(std::shared_ptr<const SwitchedModelModeScheduleManager> modeScheduleManagerPtr,
                                 PinocchioInterface pinocchioInterface, CentroidalModelPinocchioMapping<scalar_t> pinocchioMapping,
                                 const std::string& taskFile)

    : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)),
      pinocchioInterface_(std::move(pinocchioInterface)),
      pinocchioMapping_(std::move(pinocchioMapping)) {
  pinocchioMapping_.setPinocchioInterface(pinocchioInterface_);

  // Intermediate costs
  auto baseTrackingCost = getBaseTrackingCost(taskFile);

  stateInputCostCollection_.add("BaseTracking", std::move(baseTrackingCost));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotCost::LeggedRobotCost(const LeggedRobotCost& rhs)
    : BASE(rhs),
      modeScheduleManagerPtr_(rhs.modeScheduleManagerPtr_),
      pinocchioInterface_(rhs.pinocchioInterface_),
      pinocchioMapping_(rhs.pinocchioMapping_),
      stateInputCostCollection_(rhs.stateInputCostCollection_),
      stateCostCollection_(rhs.stateCostCollection_) {
  pinocchioMapping_.setPinocchioInterface(pinocchioInterface_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotCost::setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) {
  costDesiredTrajectoriesPtr_ = costDesiredTrajectoriesPtr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t LeggedRobotCost::cost(scalar_t t, const vector_t& x, const vector_t& u) {
  scalar_t cost = stateInputCostCollection_.getValue(t, x, u, *costDesiredTrajectoriesPtr_);
  cost += stateCostCollection_.getValue(t, x, *costDesiredTrajectoriesPtr_);
  return cost;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t LeggedRobotCost::finalCost(scalar_t t, const vector_t& x) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation LeggedRobotCost::costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
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
ScalarFunctionQuadraticApproximation LeggedRobotCost::finalCostQuadraticApproximation(scalar_t t, const vector_t& x) {
  return ScalarFunctionQuadraticApproximation::Zero(x.size(), 0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotCost::initializeInputCostWeight(const std::string& taskFile, matrix_t& R) {
  state_vector_t initialState;
  loadData::loadEigenMatrix(taskFile, "initialState", initialState);

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto q = pinocchioMapping_.getPinocchioJointPosition(initialState);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  Eigen::Matrix<scalar_t, 3 * FOOT_CONTACTS_NUM_, 12> baseToFeetJacobians;
  for (size_t i = 0; i < FOOT_CONTACTS_NUM_; i++) {
    matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, GENERALIZED_VEL_NUM_);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(CONTACT_POINTS_NAMES_[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                jacobianWorldToContactPointInWorldFrame);

    baseToFeetJacobians.block(3 * i, 0, 3, 12) = (jacobianWorldToContactPointInWorldFrame.topRows<3>()).block(0, BASE_DOF_NUM_, 3, 12);
  }

  R.block(TOTAL_CONTACTS_DIM_, TOTAL_CONTACTS_DIM_, 12, 12) =
      (baseToFeetJacobians.transpose() * R.block(TOTAL_CONTACTS_DIM_, TOTAL_CONTACTS_DIM_, 12, 12) * baseToFeetJacobians).eval();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotCost::getBaseTrackingCost(const std::string& taskFile) {
  matrix_t Q(STATE_DIM_, STATE_DIM_);
  matrix_t R(INPUT_DIM_, INPUT_DIM_);

  std::cerr << "\n #### Base Tracking Cost Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  loadData::loadEigenMatrix(taskFile, "R", R);
  initializeInputCostWeight(taskFile, R);
  std::cerr << "Q:  \n" << Q << std::endl;
  std::cerr << "R:  \n" << R << '\n';
  std::cerr << " #### =============================================================================" << std::endl;

  return std::unique_ptr<StateInputCost>(new LeggedRobotStateInputQuadraticCost(modeScheduleManagerPtr_, taskFile, Q, R));
}

}  // namespace legged_robot
}  // namespace ocs2
