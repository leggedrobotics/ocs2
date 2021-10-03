/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/Types.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <hpp/fcl/shape/geometric_shapes.h>

namespace ocs2 {

/**
 * Sphere approximation implmentation.
 *
 * This class approximates the collision primitive geometry: box, cylinder, and sphere, with spheres. This class is to be used with
 * pinocchio::GeometryModel.
 *
 * Reference:
 * [1] A. Voelz and K. Graichen, "Computation of Collision Distance and Gradient using an Automatic Sphere Approximation of the Robot Model
 * with Bounded Error," ISR 2018; 50th International Symposium on Robotics, 2018, pp. 1-8.
 */
class SphereApproximation {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  /** Constructor
   * @param [in] geomObjectId : index of the geometry object in GeoemtryModel
   * @param [in] geometryPtr : pointer to the geometry stored in GeometryModel
   * @param [in] maxExcess : maximum allowed excess from the object surface to the sphere surface
   */
  SphereApproximation(const size_t geomObjectId, const hpp::fcl::CollisionGeometry* geometryPtr, const scalar_t maxExcess);

  /** Get the index of the geometry object stored in GeometryModel */
  size_t getGeomObjId() const { return geomObjId_; };

  /** Get the maximum alloowed excess from the the object surface to the sphere surface */
  scalar_t getMaxExcess() const { return maxExcess_; };

  /** Get the number of spheres approximating the object */
  size_t getNumSpheres() const { return numSpheres_; };

  /** Get the radius of the spheres approximating the object */
  scalar_t getSphereRadius() const { return sphereRadius_; };

  /** Get the positions of the sphere centers w.r.t the object center */
  std::vector<vector3_t> getSphereCentersToObjectCenter() const { return sphereCentersToObjectCenter_; };

 private:
  void approximateBox(const vector_t& sides);
  void approximateCylinder(const scalar_t radius, const scalar_t length);
  void approximateRectanglularCrossSection(const vector_t& sides, const size_array_t& idxSorted, const scalar_t maxExcess,
                                           scalar_t& sphereRadius, vector_t& numSpheres, vector_t& distances);
  bool approximateCircleBase(const scalar_t radiusBase, const scalar_t radiusSphereCrossSection, const scalar_t maxExcessR, scalar_t& shift,
                             scalar_t& alpha, scalar_t& numCircles);

  const size_t geomObjId_;
  const scalar_t maxExcess_;

  size_t numSpheres_;
  scalar_t sphereRadius_;
  std::vector<vector3_t> sphereCentersToObjectCenter_;
};

}  // namespace ocs2
