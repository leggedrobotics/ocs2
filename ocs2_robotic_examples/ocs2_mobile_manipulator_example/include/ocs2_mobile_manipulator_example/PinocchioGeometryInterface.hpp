/*
 * PinocchioGeometryInterface.hpp
 *
 *  Created on: 25 Aug 2020
 *      Author: perry
 */

#pragma once

#include <ocs2_mobile_manipulator_example/PinocchioInterface.h>

#include <hpp/fcl/collision_data.h>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

namespace mobile_manipulator {

class PinocchioGeometryInterface {
 public:
  PinocchioGeometryInterface(const std::string& urdfPath, PinocchioInterface<double>& pinocchioInterface,
                             const pinocchio::GeometryModel::CollisionPairVector collisionPairs);
  virtual ~PinocchioGeometryInterface() = default;

  // TODO(perry) discussion over appropriate depth of copy/clone (ie should the interface ptrs or interfaces be copied)
  PinocchioGeometryInterface(const PinocchioGeometryInterface& other);

  pinocchio::GeometryModel& getGeometryModel() { return geometryModel_; }
  const pinocchio::GeometryModel& getGeometryModel() const { return geometryModel_; }

  std::vector<hpp::fcl::DistanceResult> computeDistances(const Eigen::Matrix<double, Eigen::Dynamic, 1>& q);

 private:
  PinocchioInterface<double> pinocchioInterface_;

  pinocchio::GeometryModel geometryModel_;
};

}  // namespace mobile_manipulator
