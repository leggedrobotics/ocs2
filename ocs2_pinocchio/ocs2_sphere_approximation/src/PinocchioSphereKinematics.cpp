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
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include "ocs2_sphere_approximation/PinocchioSphereKinematics.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematics::PinocchioSphereKinematics(PinocchioSphereInterface pinocchioSphereInterface,
                                                     const PinocchioStateInputMapping<scalar_t>& mapping)
    : pinocchioInterfacePtr_(nullptr), pinocchioSphereInterface_(std::move(pinocchioSphereInterface)), mappingPtr_(mapping.clone()) {
  linkIds_.resize(pinocchioSphereInterface_.getNumSpheresInTotal());
  const std::vector<std::string>& collisionLinkOfEachPrimitiveShape = pinocchioSphereInterface_.getCollisionLinkOfEachPrimitveShape();
  const auto numSpheres = pinocchioSphereInterface_.getNumSpheres();
  size_t count = 0;
  for (size_t i = 0; i < pinocchioSphereInterface.getNumPrimitiveShapes(); i++) {
    std::fill(linkIds_.begin() + count, linkIds_.begin() + count + numSpheres[i], collisionLinkOfEachPrimitiveShape[i]);
    count += numSpheres[i];
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematics::PinocchioSphereKinematics(const PinocchioSphereKinematics& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      pinocchioInterfacePtr_(nullptr),
      mappingPtr_(rhs.mappingPtr_->clone()),
      pinocchioSphereInterface_(rhs.pinocchioSphereInterface_),
      linkIds_(rhs.linkIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematics* PinocchioSphereKinematics::clone() const {
  return new PinocchioSphereKinematics(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioSphereKinematics::getPosition(const vector_t& state) const -> std::vector<vector3_t> {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioSphereKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  return pinocchioSphereInterface_.computeSphereCentersInWorldFrame(*pinocchioInterfacePtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioSphereKinematics::getPositionLinearApproximation(const vector_t& state) const {
  if (pinocchioInterfacePtr_ == nullptr) {
    throw std::runtime_error("[PinocchioSphereKinematics] pinocchioInterfacePtr_ is not set. Use setPinocchioInterface()");
  }

  std::vector<vector3_t> sphereCentersInWorldFrame = pinocchioSphereInterface_.computeSphereCentersInWorldFrame(*pinocchioInterfacePtr_);

  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  // const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  // TODO(mspieler): Need to copy here because getFrameJacobian() modifies data. Will be fixed in pinocchio version 3.
  pinocchio::Data data = pinocchio::Data(pinocchioInterfacePtr_->getData());

  std::vector<VectorFunctionLinearApproximation> positions;

  const auto& geometryModel = pinocchioSphereInterface_.getGeometryModel();
  const size_t numPrimitiveShapes = pinocchioSphereInterface_.getNumPrimitiveShapes();
  const auto geomObjIds = pinocchioSphereInterface_.getGeomObjIds();
  const auto numSpheres = pinocchioSphereInterface_.getNumSpheres();

  size_t count = 0;
  for (size_t i = 0; i < numPrimitiveShapes; i++) {
    const auto& parentJointId = geometryModel.geometryObjects[geomObjIds[i]].parentJoint;
    const vector3_t& jointPosition = data.oMi[parentJointId].translation();
    matrix_t jointJacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getJointJacobian(model, data, parentJointId, rf, jointJacobian);

    for (size_t j = 0; j < numSpheres[i]; j++) {
      VectorFunctionLinearApproximation pos;
      pos.f = sphereCentersInWorldFrame[count + j];
      const vector3_t sphereCenterOffset = sphereCentersInWorldFrame[count + j] - jointPosition;
      const matrix_t sphereCenterJacobian =
          jointJacobian.topRows<3>() - skewSymmetricMatrix(sphereCenterOffset) * jointJacobian.bottomRows<3>();
      std::tie(pos.dfdx, std::ignore) = mappingPtr_->getOcs2Jacobian(state, sphereCenterJacobian.topRows<3>(), matrix_t::Zero(0, model.nq));
      positions.emplace_back(std::move(pos));
    }
    count += numSpheres[i];
  }

  return positions;
}

}  // namespace ocs2
