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

#include <ocs2_sphere_approximation/SphereApproximation.h>

#include <hpp/fcl/collision_data.h>

/* Forward declaration of pinocchio geometry types */
namespace pinocchio {
struct GeometryModel;
}  // namespace pinocchio

namespace ocs2 {

class PinocchioSphereInterface final {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  PinocchioSphereInterface(const std::string& urdfPath, const PinocchioInterface& pinocchioInterface,
                           std::vector<std::string> collisionLinks, std::vector<scalar_t> maxExcesses);

  void setSphereTransforms(const PinocchioInterface& pinocchioInterface);

  std::vector<vector3_t> computeSphereCentersInWorldFrame(const PinocchioInterface& pinocchioInterface) const;

  const std::vector<std::string>& getCollisionLinks() const { return collisionLinks_; };

  const std::vector<SphereApproximation>& getSphereApproximations() const { return sphereApproximations_; }

  size_t getNumApproximations() const { return numApproximations_; };

  size_t getNumSpheresInTotal() const { return numSpheresInTotal_; };

  size_array_t getNumSpheres() const { return numSpheres_; };

  size_array_t getGeomObjIds() const { return geomObjIds_; };

  scalar_array_t getSphereRadii() const { return sphereRadii_; };

  std::vector<vector3_t> getSphereCentersToObjectCenter(size_t approxId) const {
    return sphereApproximations_[approxId].getSphereCentersToObjectCenter();
  };

  /** Access the pinocchio geometry model */
  pinocchio::GeometryModel& getGeometryModel() { return *geometryModelPtr_; }
  const pinocchio::GeometryModel& getGeometryModel() const { return *geometryModelPtr_; }

 private:
  std::shared_ptr<pinocchio::GeometryModel> geometryModelPtr_;

  // Sphere approximation for environment collision
  std::vector<std::string> collisionLinks_;
  std::vector<SphereApproximation> sphereApproximations_;
  size_t numApproximations_ = 0;
  size_t numSpheresInTotal_ = 0;
  size_array_t numSpheres_;
  size_array_t geomObjIds_;
  scalar_array_t sphereRadii_;
};

}  // namespace ocs2
