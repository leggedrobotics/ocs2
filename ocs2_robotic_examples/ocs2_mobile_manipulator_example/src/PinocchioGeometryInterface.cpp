/*
 * PinocchioGeometryInterface.cpp
 *
 *  Created on: 25 Aug 2020
 *      Author: perry
 */

#include <ocs2_mobile_manipulator_example/PinocchioGeometryInterface.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/multibody/fcl.hpp"

namespace mobile_manipulator {

PinocchioGeometryInterface::PinocchioGeometryInterface(const std::string& urdfPath, PinocchioInterface<double>& pinocchioInterface,
                                                       const pinocchio::GeometryModel::CollisionPairVector collisionPairs)
    : pinocchioInterface_(pinocchioInterface) {
  pinocchio::urdf::buildGeom(pinocchioInterface_.getModel(), urdfPath, pinocchio::COLLISION, geometryModel_);

  for (const pinocchio::CollisionPair& collisionPair : collisionPairs) {
    geometryModel_.addCollisionPair(collisionPair);
  }
}

PinocchioGeometryInterface::PinocchioGeometryInterface(const PinocchioGeometryInterface& other)
    : pinocchioInterface_(other.pinocchioInterface_), geometryModel_(other.geometryModel_) {}

std::vector<hpp::fcl::DistanceResult> PinocchioGeometryInterface::computeDistances(const Eigen::Matrix<double, Eigen::Dynamic, 1>& q) {
  pinocchio::GeometryData geometryData(geometryModel_);

  // This function performs forward kinematics
  // In the future, calling updateGeomtryPlacements without 'q' will avoid the extra forward kinematics call
  pinocchio::updateGeometryPlacements(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), geometryModel_, geometryData, q);

  pinocchio::computeDistances(geometryModel_, geometryData);

  return geometryData.distanceResults;
}

}  // namespace mobile_manipulator
