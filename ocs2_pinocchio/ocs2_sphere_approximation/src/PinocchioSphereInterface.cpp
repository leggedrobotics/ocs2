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
                                                   std::vector<std::string> collisionLinks, std::vector<scalar_t> maxExcesses)
    : geometryModelPtr_(new pinocchio::GeometryModel), collisionLinks_(std::move(collisionLinks)) {
  pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfPath, pinocchio::COLLISION, *geometryModelPtr_);

  for (size_t i = 0; i < collisionLinks_.size(); i++) {
    const auto& link = collisionLinks_[i];
    for (size_t j = 0; j < geometryModelPtr_->geometryObjects.size(); ++j) {
      const pinocchio::GeometryObject& object = geometryModelPtr_->geometryObjects[j];
      const std::string parentFrameName = pinocchioInterface.getModel().frames[object.parentFrame].name;
      if (parentFrameName == link) {
        sphereApproximations_.emplace_back(j, object.geometry.get(), maxExcesses[i]);
      }
    }
  }

  numApproximations_ = sphereApproximations_.size();
  numSpheres_.reserve(numApproximations_);
  geomObjIds_.reserve(numApproximations_);
  for (const auto& sphereApprox : sphereApproximations_) {
    const size_t numSpheres = sphereApprox.getNumSpheres();
    numSpheresInTotal_ += numSpheres;
    numSpheres_.emplace_back(numSpheres);
    geomObjIds_.emplace_back(sphereApprox.getGeomObjId());

    for (size_t j = 0; j < numSpheres; j++) {
      sphereRadii_.push_back(sphereApprox.getSphereRadius());
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto PinocchioSphereInterface::computeSphereCentersInWorldFrame(const PinocchioInterface& pinocchioInterface) const
    -> std::vector<vector3_t> {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  pinocchio::updateGeometryPlacements(pinocchioInterface.getModel(), pinocchioInterface.getData(), *geometryModelPtr_, geometryData);

  std::vector<vector3_t> sphereCentersInWorldFrame(numSpheresInTotal_);

  size_t count = 0;
  for (size_t i = 0; i < numApproximations_; i++) {
    const auto& objTransform = geometryData.oMg[geomObjIds_[i]];
    const auto& sphereCentersToObjectCenter = sphereApproximations_[i].getSphereCentersToObjectCenter();
    const size_t numSpheres = numSpheres_[i];
    for (size_t j = 0; j < numSpheres; j++) {
      sphereCentersInWorldFrame[count + j] = objTransform.rotation() * sphereCentersToObjectCenter[j] + objTransform.translation();
    }
    count += numSpheres;
  }
  return sphereCentersInWorldFrame;
}

}  // namespace ocs2
