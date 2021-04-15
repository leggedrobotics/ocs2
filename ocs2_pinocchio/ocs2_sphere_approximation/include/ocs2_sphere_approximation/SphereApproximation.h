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

//#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>

namespace ocs2 {
class SphereApproximation {
 public:
  SphereApproximation(const hpp::fcl::CollisionGeometry* geometryPtr, scalar_t maxExtrusion);

  void approximatePrimitive(const hpp::fcl::CollisionGeometry* geometryPtr);

  void setTranslation();

  scalar_t getMaxExtrusion() const { return maxExtrusion_; };
  scalar_t getSphereRadius() const { return sphereRadius_; };
  vector_array_t getSphereCentersInObjectFrame() const { return sphereCentersInObjectFrame_; };
  vector_array_t getSphereCentersInWorldFrame() const { return sphereCentersInWorldFrame_; };

 private:
  void approximateBox(const vector_t& sides);
  void approximateCylinder(const scalar_t& radius, const scalar_t& length);

  const scalar_t maxExtrusion_;

  scalar_t sphereRadius_;
  vector_array_t sphereCentersInObjectFrame_;
  vector_array_t sphereCentersInWorldFrame_;
};

}  // namespace ocs2
