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

#include <urdf_parser/urdf_parser.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioGeometryInterface::PinocchioGeometryInterface(const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
    : geometryModelPtr_(new pinocchio::GeometryModel) {
  buildGeomFromPinocchioInterface(pinocchioInterface, *geometryModelPtr_);

  addCollisionObjectPairs(pinocchioInterface, collisionObjectPairs);
}

PinocchioGeometryInterface::PinocchioGeometryInterface(const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs)
    : geometryModelPtr_(new pinocchio::GeometryModel) {
  buildGeomFromPinocchioInterface(pinocchioInterface, *geometryModelPtr_);

  addCollisionObjectPairs(pinocchioInterface, collisionObjectPairs);
  addCollisionLinkPairs(pinocchioInterface, collisionLinkPairs);
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
size_t PinocchioGeometryInterface::getNumCollisionPairs() const {
  return geometryModelPtr_->collisionPairs.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void PinocchioGeometryInterface::buildGeomFromPinocchioInterface(const PinocchioInterface& pinocchioInterface,
                                                                 pinocchio::GeometryModel& geomModel) {
  if (!pinocchioInterface.getUrdfModelPtr()) {
    throw std::runtime_error("The PinocchioInterface passed to PinocchioGeometryInterface(...) does not contain a urdf model!");
  }

  // TODO: Replace with pinocchio function that uses the ModelInterface directly
  // As of 19-04-21 there is no buildGeom that takes a ModelInterface, so we deconstruct the modelInterface into a std::stringstream first
  const std::unique_ptr<const TiXmlDocument> urdfAsXml(urdf::exportURDF(*pinocchioInterface.getUrdfModelPtr()));
  TiXmlPrinter printer;
  urdfAsXml->Accept(&printer);
  const std::stringstream urdfAsStringStream(printer.Str());

  pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfAsStringStream, pinocchio::COLLISION, geomModel);
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioGeometryInterface::addCollisionObjectPairs(const PinocchioInterface& pinocchioInterface,
                                                         const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs) {
  for (const auto& pair : collisionObjectPairs) {
    geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{pair.first, pair.second});
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PinocchioGeometryInterface::addCollisionLinkPairs(const PinocchioInterface& pinocchioInterface,
                                                       const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs) {
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

}  // namespace ocs2
