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

#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioGeometryInterface::PinocchioGeometryInterface(const std::string& urdfPath, const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
    : geometryModelPtr_(new pinocchio::GeometryModel) {
  pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfPath, pinocchio::COLLISION, *geometryModelPtr_);

  for (const auto& pair : collisionObjectPairs) {
    geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{pair.first, pair.second});
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioGeometryInterface::PinocchioGeometryInterface(const std::string& urdfPath, const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
    : geometryModelPtr_(new pinocchio::GeometryModel) {
  pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfPath, pinocchio::COLLISION, *geometryModelPtr_);

  for (const auto& pair : collisionObjectPairs) {
    geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{pair.first, pair.second});
  }

  for (const auto& linkPair : collisionLinkPairs) {
    bool addedPair = false;
    for (size_t i = 0; i < geometryModelPtr_->geometryObjects.size(); ++i) {
      const pinocchio::GeometryObject& object1 = geometryModelPtr_->geometryObjects[i];
      const std::string parentFrameName1 = pinocchioInterface.getModel().frames[object1.parentFrame].name;
      if (parentFrameName1 == linkPair.first) {
        for (size_t j = 0; j < geometryModelPtr_->geometryObjects.size(); ++j) {
          const pinocchio::GeometryObject& object2 = geometryModelPtr_->geometryObjects[j];
          const std::string parentFrameName2 = pinocchioInterface.getModel().frames[object2.parentFrame].name;
          if (parentFrameName2 == linkPair.second) {
            geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{i, j});
            addedPair = true;
          }
        }
      }
    }
    if (!addedPair) {
      std::cerr << "WARNING: in collision link pair [" << linkPair.first << ", " << linkPair.second
                << "], one or both of the links don't exist in the pinocchio/urdf model\n";
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioGeometryInterface::PinocchioGeometryInterface(const std::string& urdfPath, const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::string> sphereApproximationLinks, vector_t maxExtrusions)
    : geometryModelPtr_(new pinocchio::GeometryModel), maxExtrusions_(std::move(maxExtrusions)) {
  pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfPath, pinocchio::COLLISION, *geometryModelPtr_);

  std::cerr << "Finding objects ...\n";
  size_t linkCount = 0;
  for (const auto& link : sphereApproximationLinks) {
    for (size_t i = 0; i < geometryModelPtr_->geometryObjects.size(); ++i) {
      const pinocchio::GeometryObject& object = geometryModelPtr_->geometryObjects[i];
      const std::string parentFrameName = pinocchioInterface.getModel().frames[object.parentFrame].name;
      if (parentFrameName == link) {
        std::cerr << "Found one object in frame:" << link << "\n";
        sphereApproximations_.emplace_back(i, object.geometry.get(), maxExtrusions_[linkCount]);
      }
    }
    linkCount++;
  }
  std::cerr << "Found and approximated " << sphereApproximations_.size() << " objects\n";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::vector<hpp::fcl::DistanceResult> PinocchioGeometryInterface::computeDistances(const PinocchioInterface& pinocchioInterface) const {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  pinocchio::updateGeometryPlacements(pinocchioInterface.getModel(), pinocchioInterface.getData(), *geometryModelPtr_, geometryData);
  pinocchio::computeDistances(*geometryModelPtr_, geometryData);

  return std::move(geometryData.distanceResults);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioGeometryInterface::setSphereTransforms(const PinocchioInterface& pinocchioInterface) {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  pinocchio::updateGeometryPlacements(pinocchioInterface.getModel(), pinocchioInterface.getData(), *geometryModelPtr_, geometryData);

  for (auto& sphereApprox : sphereApproximations_) {
    const auto& objectTransform = geometryData.oMg[sphereApprox.getObjectId()];
    sphereApprox.setSphereTransforms(objectTransform.rotation(), objectTransform.translation());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t PinocchioGeometryInterface::getNumCollisionPairs() const {
  return geometryModelPtr_->collisionPairs.size();
}

}  // namespace ocs2
