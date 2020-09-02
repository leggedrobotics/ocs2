/*
 * PinocchioGeometryInterface.hpp
 *
 *  Created on: 25 Aug 2020
 *      Author: perry
 */

#pragma once

#include <ocs2_mobile_manipulator_example/PinocchioInterface.h>

#include <hpp/fcl/collision_data.h>

/* Forward declaration of pinocchio geometry types */
namespace pinocchio {
struct GeometryModel;
}  // namespace pinocchio

namespace ocs2 {

class PinocchioGeometryInterface {
 public:
  PinocchioGeometryInterface(const std::string& urdfPath, PinocchioInterface<scalar_t>& pinocchioInterface,
                             const std::vector<std::pair<size_t, size_t>>& collisionPairs);
  virtual ~PinocchioGeometryInterface() = default;

  // TODO(perry) discussion over appropriate depth of copy/clone (ie should the interface ptrs or interfaces be copied)
  PinocchioGeometryInterface(const PinocchioGeometryInterface& other);

  pinocchio::GeometryModel& getGeometryModel() { return *geometryModelPtr_; }
  const pinocchio::GeometryModel& getGeometryModel() const { return *geometryModelPtr_; }

  std::vector<hpp::fcl::DistanceResult> computeDistances(const Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>& q);

 private:
  PinocchioInterface<scalar_t> pinocchioInterface_;
  std::shared_ptr<pinocchio::GeometryModel> geometryModelPtr_;
};

}  // namespace ocs2
