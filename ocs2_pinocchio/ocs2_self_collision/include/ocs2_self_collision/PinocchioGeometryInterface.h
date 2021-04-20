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

#pragma once

#include <utility>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <hpp/fcl/collision_data.h>

#include <urdf_model/model.h>

/* Forward declaration of pinocchio geometry types */
namespace pinocchio {
struct GeometryModel;
}  // namespace pinocchio

namespace ocs2 {

class PinocchioGeometryInterface final {
 public:
  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] collisionObjectPairs: List of collision object index pairs
   */
  PinocchioGeometryInterface(const PinocchioInterface& pinocchioInterface,
                             const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs);

  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] collisionLinkPairs: List of collision link pairs by string name. One link can contain multiple colision objects.
   *                                 In this case, all collision object combinations are added.
   * @param [in] collisionObjectPairs: List of collision object index pairs
   */
  PinocchioGeometryInterface(const PinocchioInterface& pinocchioInterface,
                             const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
                             const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs = std::vector<std::pair<size_t, size_t>>());

  /**
   * Compute collision pair distances
   *
   * @note Requires pinocchioInterface with updated joint placements by calling forwardKinematics().
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @return An array of distances between pairs of collision bodies defined in the constructor.
   */
  std::vector<hpp::fcl::DistanceResult> computeDistances(const PinocchioInterface& pinocchioInterface) const;

  /** Get the number of collision pairs */
  size_t getNumCollisionPairs() const;

  /** Access the pinocchio geometry model */
  pinocchio::GeometryModel& getGeometryModel() { return *geometryModelPtr_; }
  const pinocchio::GeometryModel& getGeometryModel() const { return *geometryModelPtr_; }

 private:
  // Construction helpers
  void buildGeomFromPinocchioInterface(const PinocchioInterface& pinocchioInterface, pinocchio::GeometryModel& geomModel);
  void addCollisionObjectPairs(const PinocchioInterface& pinocchioInterface,
                               const std::vector<std::pair<size_t, size_t>>& collisionObjectPairs);
  void addCollisionLinkPairs(const PinocchioInterface& pinocchioInterface,
                             const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs);

  std::shared_ptr<pinocchio::GeometryModel> geometryModelPtr_;
};

}  // namespace ocs2
