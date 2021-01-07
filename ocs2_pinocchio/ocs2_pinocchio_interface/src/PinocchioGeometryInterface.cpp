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

/*
 * PinocchioGeometryInterface.cpp
 *
 *  Created on: 25 Aug 2020
 *      Author: perry
 */

#include <pinocchio/fwd.hpp>

#include <ocs2_pinocchio/PinocchioGeometryInterface.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/fcl.hpp"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioGeometryInterface::PinocchioGeometryInterface(const std::string& urdfPath, PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionPairs)
    : pinocchioInterface_(pinocchioInterface), geometryModelPtr_(new pinocchio::GeometryModel) {
  pinocchio::urdf::buildGeom(pinocchioInterface_.getModel(), urdfPath, pinocchio::COLLISION, *geometryModelPtr_);

  for (const auto& pair : collisionPairs) {
    geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{pair.first, pair.second});
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioGeometryInterface::PinocchioGeometryInterface(const std::string& urdfPath, PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
    : pinocchioInterface_(pinocchioInterface), geometryModelPtr_(new pinocchio::GeometryModel) {
  pinocchio::urdf::buildGeom(pinocchioInterface_.getModel(), urdfPath, pinocchio::COLLISION, *geometryModelPtr_);

  for (const auto& pair : collisionObjectPairs) {
    geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{pair.first, pair.second});
  }

  for (const auto& linkPair : collisionLinkPairs) {
    bool addedPair = false;
    for (size_t i = 0; i < geometryModelPtr_->geometryObjects.size(); ++i) {
      const pinocchio::GeometryObject& object1 = geometryModelPtr_->geometryObjects[i];
      const std::string parentFrameName1 = pinocchioInterface_.getModel().frames[object1.parentFrame].name;
      if (parentFrameName1 == linkPair.first) {
        for (size_t j = 0; j < geometryModelPtr_->geometryObjects.size(); ++j) {
          const pinocchio::GeometryObject& object2 = geometryModelPtr_->geometryObjects[j];
          const std::string parentFrameName2 = pinocchioInterface_.getModel().frames[object2.parentFrame].name;
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
PinocchioGeometryInterface::PinocchioGeometryInterface(const PinocchioGeometryInterface& other) = default;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<hpp::fcl::DistanceResult> PinocchioGeometryInterface::computeDistances(const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& q) {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  // This function performs forward kinematics
  // In the future, calling updateGeomtryPlacements without 'q' will avoid the extra forward kinematics call
  pinocchio::updateGeometryPlacements(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), *geometryModelPtr_, geometryData, q);

  pinocchio::computeDistances(*geometryModelPtr_, geometryData);

  return geometryData.distanceResults;
}

}  // namespace ocs2
