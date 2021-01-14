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

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics::PinocchioEndEffectorKinematics(PinocchioInterface pinocchioInterface,
                                                               const PinocchioStateInputMapping<scalar_t>& mapping,
                                                               std::vector<std::string> endEffectorIds)
    : pinocchioInterface_(std::move(pinocchioInterface)), mappingPtr_(mapping.clone()), endEffectorIds_(std::move(endEffectorIds)) {
  for (const auto& bodyName : endEffectorIds_) {
    endEffectorFrameIds_.push_back(pinocchioInterface_.getModel().getBodyId(bodyName));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioEndEffectorKinematics::PinocchioEndEffectorKinematics(const PinocchioEndEffectorKinematics& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      pinocchioInterface_(rhs.pinocchioInterface_),
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
  const pinocchio::Model& model = pinocchioInterface_.getModel();
  pinocchio::Data& data = pinocchioInterface_.getData();
  const vector_t q = mappingPtr_->getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  std::vector<vector3_t> positions;
  for (const auto& frameId : endEffectorFrameIds_) {
    positions.push_back(data.oMf[frameId].translation());
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioEndEffectorKinematics::getVelocities(const vector_t& state, const vector_t& input) -> std::vector<vector3_t> {
  const pinocchio::Model& model = pinocchioInterface_.getModel();
  pinocchio::Data& data = pinocchioInterface_.getData();
  const vector_t q = mappingPtr_->getPinocchioJointPosition(state);
  const vector_t v = mappingPtr_->getPinocchioJointVelocity(state, input);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD;

  pinocchio::forwardKinematics(model, data, q, v);

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
  const pinocchio::Model& model = pinocchioInterface_.getModel();
  pinocchio::Data& data = pinocchioInterface_.getData();
  const vector_t q = mappingPtr_->getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, q);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD;

  std::vector<VectorFunctionLinearApproximation> positions;
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);

    VectorFunctionLinearApproximation pos;
    pos.f = data.oMf[frameId].translation();
    std::tie(pos.dfdx, pos.dfdu) = mappingPtr_->getOcs2Jacobian(state, J.topRows(3), matrix_t::Zero(3, model.nv));
    positions.emplace_back(std::move(pos));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioEndEffectorKinematics::getVelocitiesLinearApproximation(const vector_t& state,
                                                                                                                const vector_t& input) {
  const pinocchio::Model& model = pinocchioInterface_.getModel();
  pinocchio::Data& data = pinocchioInterface_.getData();
  const vector_t q = mappingPtr_->getPinocchioJointPosition(state);
  const vector_t v = mappingPtr_->getPinocchioJointVelocity(state, input);

  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::computeJointJacobiansTimeVariation(model, data, q, v);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD;

  std::vector<VectorFunctionLinearApproximation> velocities;
  for (const auto& frameId : endEffectorFrameIds_) {
    matrix_t J = matrix_t::Zero(6, model.nq);
    pinocchio::getFrameJacobian(model, data, frameId, rf, J);

    matrix_t dJ = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameJacobianTimeVariation(model, data, frameId, rf, dJ);

    VectorFunctionLinearApproximation vel;
    vel.f = pinocchio::getFrameVelocity(model, data, frameId, rf).linear();
    std::tie(vel.dfdx, vel.dfdu) = mappingPtr_->getOcs2Jacobian(state, dJ.topRows(3), J.topRows(3));
    velocities.emplace_back(std::move(vel));
  }
  return velocities;
}

}  // namespace ocs2
