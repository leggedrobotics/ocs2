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
  PinocchioGeometryInterface(const std::string& urdfPath, PinocchioInterface<double> pinocchioInterface,
                             const pinocchio::GeometryModel::CollisionPairVector collisionPairs);
  virtual ~PinocchioGeometryInterface() = default;

  pinocchio::GeometryModel& getGeometryModel() { return *geometryModel_; }
  const pinocchio::GeometryModel& getGeometryModel() const { return *geometryModel_; }

  std::vector<hpp::fcl::DistanceResult> computeDistances(const Eigen::Matrix<double, Eigen::Dynamic, 1>& q);

 private:
  PinocchioInterface<double> pinocchioInterface_;

  std::shared_ptr<pinocchio::GeometryModel> geometryModel_;
};

}  // namespace mobile_manipulator
