/*
 * PinocchioGeometryInterface.cpp
 *
 *  Created on: 25 Aug 2020
 *      Author: perry
 */

#include <pinocchio/fwd.hpp>

#include <ocs2_mobile_manipulator_example/PinocchioGeometryInterface.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/fcl.hpp"

namespace ocs2 {

PinocchioGeometryInterface::PinocchioGeometryInterface(const std::string& urdfPath, PinocchioInterface<scalar_t>& pinocchioInterface,
                                                       const std::vector<std::pair<size_t, size_t>>& collisionPairs)
    : pinocchioInterface_(pinocchioInterface), geometryModelPtr_(new pinocchio::GeometryModel) {
  pinocchio::urdf::buildGeom(pinocchioInterface_.getModel(), urdfPath, pinocchio::COLLISION, *geometryModelPtr_);

  for (const auto& pair : collisionPairs) {
    geometryModelPtr_->addCollisionPair(pinocchio::CollisionPair{pair.first, pair.second});
  }
}

PinocchioGeometryInterface::PinocchioGeometryInterface(const PinocchioGeometryInterface& other) = default;

std::vector<hpp::fcl::DistanceResult> PinocchioGeometryInterface::computeDistances(const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& q) {
  pinocchio::GeometryData geometryData(*geometryModelPtr_);

  // This function performs forward kinematics
  // In the future, calling updateGeomtryPlacements without 'q' will avoid the extra forward kinematics call
  pinocchio::updateGeometryPlacements(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), *geometryModelPtr_, geometryData, q);

  pinocchio::computeDistances(*geometryModelPtr_, geometryData);

  return geometryData.distanceResults;
}

}  // namespace ocs2
