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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioSphereInterface::PinocchioSphereInterface(const std::string& urdfPath, const PinocchioInterface& pinocchioInterface,
                                                   std::vector<std::string> envCollisionLinks, std::vector<scalar_t> maxExcesses)
    : geometryModelPtr_(new pinocchio::GeometryModel),
      envCollisionLinks_(std::move(envCollisionLinks)),
      maxExcesses_(std::move(maxExcesses)) {
  pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfPath, pinocchio::COLLISION, *geometryModelPtr_);

  for (size_t i = 0; i < envCollisionLinks_.size(); i++) {
    const auto& link = envCollisionLinks_[i];
    for (size_t j = 0; j < geometryModelPtr_->geometryObjects.size(); ++j) {
      const pinocchio::GeometryObject& object = geometryModelPtr_->geometryObjects[j];
      const std::string parentFrameName = pinocchioInterface.getModel().frames[object.parentFrame].name;
      if (parentFrameName == link) {
        sphereApproximations_.emplace_back(j, object.geometry.get(), maxExcesses_[i]);
      }
    }
  }

  for (const auto& sphereApprox : sphereApproximations_) {
    numSpheres_ += sphereApprox.getNumSpheres();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioSphereInterface::setSphereTransforms(const PinocchioInterface& pinocchioInterface) {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  pinocchio::updateGeometryPlacements(pinocchioInterface.getModel(), pinocchioInterface.getData(), *geometryModelPtr_, geometryData);

  for (auto& sphereApprox : sphereApproximations_) {
    const auto& objectTransform = geometryData.oMg[sphereApprox.getGeomObjectId()];
    sphereApprox.setSphereTransforms(objectTransform.rotation(), objectTransform.translation());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioSphereInterface::computeSphereCentersInWorldFrame(const PinocchioInterface& pinocchioInterface) const
    -> std::vector<vector3_t> {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  pinocchio::updateGeometryPlacements(pinocchioInterface.getModel(), pinocchioInterface.getData(), *geometryModelPtr_, geometryData);

  std::vector<vector3_t> sphereCentersInWorldFrame(numSpheres_);

  size_t count = 0;
  for (const auto& sphereApprox : sphereApproximations_) {
    const auto& objectTransform = geometryData.oMg[sphereApprox.getGeomObjectId()];
    const auto& sphereCentersToObjectCenter = sphereApprox.getSphereCentersToObjectCenter();
    const size_t numSpherePerObject = sphereApprox.getNumSpheres();
    for (size_t i = 0; i < numSpherePerObject; i++) {
      sphereCentersInWorldFrame[count + i] = objectTransform.rotation() * sphereCentersToObjectCenter[i] + objectTransform.translation();
    }
    count += numSpherePerObject;
  }
  return sphereCentersInWorldFrame;
}

}  // namespace ocs2
