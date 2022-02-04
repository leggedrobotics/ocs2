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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematicsCppAd.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematicsCppAd::PinocchioSphereKinematicsCppAd(const PinocchioInterface& pinocchioInterface,
                                                               PinocchioSphereInterface pinocchioSphereInterface,
                                                               const PinocchioStateInputMapping<ad_scalar_t>& mapping, size_t stateDim,
                                                               size_t inputDim, const std::string& modelName,
                                                               const std::string& modelFolder, bool recompileLibraries, bool verbose)
    : pinocchioSphereInterface_(std::move(pinocchioSphereInterface)) {
  const std::vector<SphereApproxParam> sphereApproxParams = createSphereApproxParams();

  linkIds_.resize(pinocchioSphereInterface_.getNumSpheresInTotal());
  const std::vector<std::string>& collisionLinkOfEachPrimitiveShape = pinocchioSphereInterface_.getCollisionLinkOfEachPrimitveShape();
  const auto numSpheres = pinocchioSphereInterface_.getNumSpheres();
  size_t count = 0;
  for (size_t i = 0; i < pinocchioSphereInterface.getNumPrimitiveShapes(); i++) {
    std::fill(linkIds_.begin() + count, linkIds_.begin() + count + numSpheres[i], collisionLinkOfEachPrimitiveShape[i]);
    count += numSpheres[i];
  }

  // initialize CppAD interface
  auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

  // position function
  auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
    y = getPositionCppAd(pinocchioInterfaceCppAd, mapping, sphereApproxParams, x);
  };
  positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, stateDim, modelName + "_position", modelFolder));

  if (recompileLibraries) {
    positionCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
  } else {
    positionCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematicsCppAd::PinocchioSphereKinematicsCppAd(const PinocchioSphereKinematicsCppAd& rhs)
    : EndEffectorKinematics<scalar_t>(rhs),
      positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)),
      pinocchioSphereInterface_(rhs.pinocchioSphereInterface_),
      linkIds_(rhs.linkIds_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereKinematicsCppAd* PinocchioSphereKinematicsCppAd::clone() const {
  return new PinocchioSphereKinematicsCppAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ad_vector_t PinocchioSphereKinematicsCppAd::getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                                             const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                             const std::vector<SphereApproxParam>& sphereApproxParams,
                                                             const ad_vector_t& state) {
  const PinocchioInterfaceCppAd::Model& model = pinocchioInterfaceCppAd.getModel();
  PinocchioInterfaceCppAd::Data& data = pinocchioInterfaceCppAd.getData();
  const ad_vector_t q = mapping.getPinocchioJointPosition(state);

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateGlobalPlacements(model, data);

  const pinocchio::GeometryModel& geometryModel = pinocchioSphereInterface_.getGeometryModel();
  const size_t numPrimitiveShapes = pinocchioSphereInterface_.getNumPrimitiveShapes();
  const size_array_t geomObjIds = pinocchioSphereInterface_.getGeomObjIds();
  const size_array_t numSpheres = pinocchioSphereInterface_.getNumSpheres();
  ad_vector_t sphereCentersInWorldFrame(3 * pinocchioSphereInterface_.getNumSpheresInTotal());

  size_t count = 0;
  for (size_t i = 0; i < numPrimitiveShapes; i++) {
    const size_t parentJointId = geometryModel.geometryObjects[geomObjIds[i]].parentJoint;
    const ad_vector_t& translation = sphereApproxParams[i].placementTranslation.cast<ad_scalar_t>();
    const Eigen::Quaternion<ad_scalar_t>& quaternion = sphereApproxParams[i].placementOrientation.cast<ad_scalar_t>();
    const std::vector<vector3_t>& sphereCentersToObjectCenter = sphereApproxParams[i].sphereCentersToObjectCenter;

    for (size_t j = 0; j < numSpheres[i]; j++) {
      sphereCentersInWorldFrame.segment<3>(count + 3 * j) = data.oMi[parentJointId].translation();
      sphereCentersInWorldFrame.segment<3>(count + 3 * j).noalias() +=
          data.oMi[parentJointId].rotation() *
          (quaternion._transformVector(sphereCentersToObjectCenter[j].cast<ad_scalar_t>()) + translation);
    }

    count += 3 * numSpheres[i];
  }

  return sphereCentersInWorldFrame;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioSphereKinematicsCppAd::getPosition(const vector_t& state) const -> std::vector<vector3_t> {
  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);

  std::vector<vector3_t> positions;
  positions.reserve(linkIds_.size());
  for (int i = 0; i < linkIds_.size(); i++) {
    positions.emplace_back(positionValues.segment<3>(3 * i));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<VectorFunctionLinearApproximation> PinocchioSphereKinematicsCppAd::getPositionLinearApproximation(const vector_t& state) const {
  const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);
  const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state);

  std::vector<VectorFunctionLinearApproximation> positions;
  positions.reserve(linkIds_.size());
  for (int i = 0; i < linkIds_.size(); i++) {
    VectorFunctionLinearApproximation pos;
    pos.f = positionValues.segment<3>(3 * i);
    pos.dfdx = positionJacobian.block(3 * i, 0, 3, state.rows());
    positions.emplace_back(std::move(pos));
  }
  return positions;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioSphereKinematicsCppAd::createSphereApproxParams() const -> std::vector<SphereApproxParam> {
  const pinocchio::GeometryModel& geometryModel = pinocchioSphereInterface_.getGeometryModel();
  const size_t numPrimitiveShapes = pinocchioSphereInterface_.getNumPrimitiveShapes();
  const size_array_t geomObjIds = pinocchioSphereInterface_.getGeomObjIds();
  std::vector<SphereApproxParam> sphereApproxParams;

  sphereApproxParams.reserve(numPrimitiveShapes);
  for (size_t i = 0; i < numPrimitiveShapes; ++i) {
    const auto& placement = geometryModel.geometryObjects[geomObjIds[i]].placement;
    sphereApproxParams.emplace_back(placement.translation(), matrixToQuaternion(placement.rotation()),
                                    pinocchioSphereInterface_.getSphereCentersToObjectCenter(i));
  }

  return sphereApproxParams;
}

}  // namespace ocs2
