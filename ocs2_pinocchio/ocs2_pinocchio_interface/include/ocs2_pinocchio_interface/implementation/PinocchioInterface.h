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

#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(const Model& model) {
  robotModelPtr_ = std::make_shared<const Model>(model);
  robotDataPtr_ = std::unique_ptr<Data>(new Data(*robotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::~PinocchioInterfaceTpl() = default;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(const PinocchioInterfaceTpl<SCALAR>& rhs)
    : robotModelPtr_(rhs.robotModelPtr_), robotDataPtr_(new Data(*rhs.robotDataPtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(PinocchioInterfaceTpl<SCALAR>&& rhs)
    : robotModelPtr_(std::move(rhs.robotModelPtr_)), robotDataPtr_(std::move(rhs.robotDataPtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>& PinocchioInterfaceTpl<SCALAR>::operator=(const PinocchioInterfaceTpl<SCALAR>& rhs) {
  robotModelPtr_ = rhs.robotModelPtr_;
  robotDataPtr_.reset(new Data(*rhs.robotDataPtr_));
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>& PinocchioInterfaceTpl<SCALAR>::operator=(PinocchioInterfaceTpl<SCALAR>&& rhs) {
  std::swap(robotModelPtr_, rhs.robotModelPtr_);
  std::swap(robotDataPtr_, rhs.robotDataPtr_);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PinocchioInterfaceTpl<SCALAR>::getBodyPosition(pinocchio::FrameIndex bodyId) const -> vector3_t {
  return robotDataPtr_->oMf[bodyId].translation();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PinocchioInterfaceTpl<SCALAR>::getBodyOrientation(pinocchio::FrameIndex bodyId) const -> quaternion_t {
  return matrixToQuaternion(robotDataPtr_->oMf[bodyId].rotation());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
size_t PinocchioInterfaceTpl<SCALAR>::getBodyId(const std::string& bodyName) const {
  return robotModelPtr_->getBodyId(bodyName);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void PinocchioInterfaceTpl<SCALAR>::forwardKinematics(const vector_t& q) {
  pinocchio::forwardKinematics(*robotModelPtr_, *robotDataPtr_, q);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void PinocchioInterfaceTpl<SCALAR>::updateFramePlacements() {
  pinocchio::updateFramePlacements(*robotModelPtr_, *robotDataPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void PinocchioInterfaceTpl<SCALAR>::updateGlobalPlacements() {
  pinocchio::updateGlobalPlacements(*robotModelPtr_, *robotDataPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
void PinocchioInterfaceTpl<SCALAR>::computeJointJacobians(const vector_t& q) {
  pinocchio::computeJointJacobians(*robotModelPtr_, *robotDataPtr_, q);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PinocchioInterfaceTpl<SCALAR>::getJointJacobian(pinocchio::JointIndex jointIndex) const -> matrix_t {
  matrix_t jointJacobian = matrix_t::Zero(6, robotModelPtr_->nv);
  pinocchio::getJointJacobian(*robotModelPtr_, *robotDataPtr_, jointIndex, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jointJacobian);
  return jointJacobian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PinocchioInterfaceTpl<SCALAR>::getJointPosition(pinocchio::JointIndex jointIndex) const -> vector3_t {
  return robotDataPtr_->oMi[jointIndex].translation();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
auto PinocchioInterfaceTpl<SCALAR>::getJointOrientation(pinocchio::JointIndex jointIndex) const -> quaternion_t {
  return matrixToQuaternion(robotDataPtr_->oMi[jointIndex].rotation());
}

}  // namespace ocs2
