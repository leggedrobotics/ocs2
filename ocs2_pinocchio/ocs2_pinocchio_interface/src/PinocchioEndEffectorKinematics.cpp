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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics::PinocchioEndEffectorKinematics(const PinocchioInterface& pinocchioInterface,
                                                               const PinocchioStateInputMapping<scalar_t>& mapping,
                                                               std::vector<std::string> endEffectorIds)
    : pinocchioInterfacePtr_(nullptr), mappingPtr_(mapping.clone()), endEffectorIds_(std::move(endEffectorIds)) {
  for (const auto& bodyName : endEffectorIds_) {
    endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getBodyId(bodyName));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics::PinocchioEndEffectorKinematics(const PinocchioEndEffectorKinematics& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      pinocchioInterfacePtr_(nullptr),
      mappingPtr_(rhs.mappingPtr_->clone()),
      endEffectorIds_(rhs.endEffectorIds_),
      endEffectorFrameIds_(rhs.endEffectorFrameIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics* PinocchioEndEffectorKinematics::clone() const {
  return new PinocchioEndEffectorKinematics(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const std::vector<std::string>& PinocchioEndEffectorKinematics::getIds() const {
  return endEffectorIds_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::getPositions(const vector_t& state) -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> positions;
  for (const auto& frameId : endEffectorFrameIds_) {
    positions.emplace_back(data.oMf[frameId].translation());
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::getPoses(const vector_t& state) -> std::vector<std::pair<vector3_t, quaternion_t>> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<std::pair<vector3_t, quaternion_t>> poses;
  for (const auto& frameId : endEffectorFrameIds_) {
    poses.emplace_back(data.oMf[frameId].translation(), matrixToQuaternion(data.oMf[frameId].rotation()));
  }
  return poses;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::getVelocities(const vector_t& state, const vector_t& input) -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> velocities;
  for (const auto& frameId : endEffectorFrameIds_) {
    velocities.emplace_back(pinocchio::getFrameVelocity(model, data, frameId, rf).linear());
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getPositionsLinearApproximation(const vector_t& state) {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> positions;
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);

    VectorFunctionLinearApproximation pos;
    pos.f = data.oMf[frameId].translation();
    std::tie(pos.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, J.topRows(3), matrix_t::Zero(0, model.nv));
    positions.emplace_back(std::move(pos));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getVelocitiesLinearApproximation(const vector_t& state,
                                                                                                                const vector_t& input) {
  // TODO(mspieler): v_partial_dq does not match CppAD reference
  throw std::runtime_error("[PinocchioEndEffectorKinematics] getVelocitiesLinearApproximation() is not implemented");

  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  std::vector<VectorFunctionLinearApproximation> velocities;
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t v_partial_dq = matrix_t::Zero(6, model.nv);
    matrix_t v_partial_dv = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameVelocityDerivatives(model, data, frameId, rf, v_partial_dq, v_partial_dv);

    VectorFunctionLinearApproximation vel;
    vel.f = pinocchio::getFrameVelocity(model, data, frameId, rf).linear();
    std::tie(vel.dfdx, vel.dfdu) = mappingPtr_->getOcs2Jacobian(state, v_partial_dq.topRows(3), v_partial_dv.topRows(3));
    velocities.emplace_back(std::move(vel));
  }
  return velocities;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::getOrientationError(const vector_t& state, const std::vector<quaternion_t>& referenceOrientations)
    -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();

  std::vector<vector3_t> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const auto frameId = endEffectorFrameIds_[i];
    errors.emplace_back(quaternionDistance(matrixToQuaternion(data.oMf[frameId].rotation()), referenceOrientations[i]));
  }
  return errors;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getOrientationErrorLinearApproximation(
    const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioEndEffectorKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> errors;
  for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
    const auto frameId = endEffectorFrameIds_[i];
    VectorFunctionLinearApproximation err;
    err.f = quaternionDistance(matrixToQuaternion(data.oMf[frameId].rotation()), referenceOrientations[i]);
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);
    std::tie(err.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, J.bottomRows(3), matrix_t::Zero(0, model.nv));
    errors.emplace_back(std::move(err));
  }
  return errors;
}

}  // namespace ocs2
