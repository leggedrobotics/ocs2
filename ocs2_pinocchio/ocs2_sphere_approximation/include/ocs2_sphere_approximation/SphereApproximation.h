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

//#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

namespace ocs2 {
class SphereApproximation {
 public:
  SphereApproximation(const size_t objectId, const hpp::fcl::CollisionGeometry* geometryPtr, const scalar_t maxExcess);

  void setSphereTransforms(const matrix_t& objectRotation, const vector_t& objectTranslation);

  size_t getObjectId() const { return objectId_; };
  scalar_t getMaxExcess() const { return maxExcess_; };
  scalar_t getSphereRadius() const { return sphereRadius_; };
  vector_array_t getSphereCentersToObjectCenter() const { return sphereCentersToObjectCenter_; };
  vector_array_t getSphereCentersInWorldFrame() const { return sphereCentersInWorldFrame_; };

 private:
  void approximateBox(const vector_t& sides);
  void approximateCylinder(const scalar_t& radius, const scalar_t& length);
  bool approximateCircleBase(const scalar_t& radiusBase, const scalar_t& radiusSphereCrossSection, const scalar_t& maxExcessR,
                             scalar_t& shift, scalar_t& alpha, scalar_t& numCircles);

  const size_t objectId_;
  const scalar_t maxExcess_;

  scalar_t sphereRadius_;
  vector_array_t sphereCentersToObjectCenter_;
  vector_array_t sphereCentersInWorldFrame_;
};

}  // namespace ocs2
