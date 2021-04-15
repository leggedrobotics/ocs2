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

#include "ocs2_sphere_approximation/SphereApproximation.h"

#include <iostream>

namespace ocs2 {

SphereApproximation::SphereApproximation(const hpp::fcl::CollisionGeometry* geometryPtr, scalar_t maxExtrusion)
    : maxExtrusion_(maxExtrusion) {
  approximatePrimitive(geometryPtr);
}

void SphereApproximation::approximatePrimitive(const hpp::fcl::CollisionGeometry* geometryPtr) {
  const auto& nodeType = geometryPtr->getNodeType();
  std::cerr << "[SphereApproximation] Node type: " << static_cast<size_t>(nodeType) << "\n";
  switch (nodeType) {
    case hpp::fcl::NODE_TYPE::GEOM_BOX: {
      std::cerr << "[SphereApproximation] Casting Geometry Ptr to Box Ptr...\n";
      const auto* boxPtr = dynamic_cast<const hpp::fcl::Box*>(geometryPtr);

      std::cerr << "[SphereApproximation] Approximating BOX with spheres...\n";
      approximateBox(boxPtr->halfSide * 2);
      break;
    }
    case hpp::fcl::NODE_TYPE::GEOM_CYLINDER: {
      std::runtime_error("[SphereApproximation] Sphere approximation of CYLINDER not yet fully implemented");

      std::cerr << "[SphereApproximation] Casting Geometry Ptr to Cylinder Ptr...\n";
      const auto* cylinderPtr = dynamic_cast<const hpp::fcl::Cylinder*>(geometryPtr);

      std::cerr << "[SphereApproximation] Approximating CYLINDER witoh spheres...\n";
      approximateCylinder(cylinderPtr->radius, cylinderPtr->halfLength * 2);
      break;
    }
    case hpp::fcl::NODE_TYPE::GEOM_SPHERE: {
      std::runtime_error("[SphereApproximation] Sphere approximation of SPHERE not yet fully implemented");
      const auto* spherePtr = dynamic_cast<const hpp::fcl::Sphere*>(geometryPtr);
      sphereRadius_ = spherePtr->radius;
      break;
    }
    default:
      throw std::runtime_error("[SphereApproximation] Undefined shape primitive for sphere approximation");
  }
}

void SphereApproximation::setTranslation() {}

void SphereApproximation::approximateBox(const vector_t& sides) {
  // indices of the shortest, medium, and longest side
  size_array_t indicesInLengthOrder(3);

  // TODO (jichiu): Figure out a more efficient way of sorting indices
  sides.minCoeff(&indicesInLengthOrder[0]);
  sides.maxCoeff(&indicesInLengthOrder[2]);
  indicesInLengthOrder[1] = 3 - indicesInLengthOrder[0] - indicesInLengthOrder[2];

  vector_t initSphereRadii(3);
  initSphereRadii << sides.norm() / 2, sides[indicesInLengthOrder[0]] / 2 + maxExtrusion_,
      std::sqrt(3) * maxExtrusion_ / (std::sqrt(3) - 1);
  size_t shortestRadiusInd;
  sphereRadius_ = initSphereRadii.minCoeff(&shortestRadiusInd);

  // Distance between the first sphere center and the corner along x, y, z
  vector_t distances(3);
  // Number of spheres along x, y, z
  vector_t numSpheres(3);

  if (shortestRadiusInd == 0) {
    std::cerr << "Case 1: One sphere for the entire box\n";
    distances = sides / 2;

    numSpheres = vector_t::Ones(3);

    // No re-calculation of the distances is required

  } else if (shortestRadiusInd == 1) {
    scalar_t dist =
        std::sqrt(sphereRadius_ * sphereRadius_ - sides[indicesInLengthOrder[0]] * sides[indicesInLengthOrder[0]] / 4) / std::sqrt(2);

    if (dist >= sides[indicesInLengthOrder[1]] / 2) {
      std::cerr << "Case 2-1: One sphere for the shortest side & One sphere for the medium side\n";
      distances[indicesInLengthOrder[0]] = sides[indicesInLengthOrder[0]] / 2;
      distances[indicesInLengthOrder[1]] = sides[indicesInLengthOrder[1]] / 2;
      distances[indicesInLengthOrder[2]] =
          std::sqrt(sphereRadius_ * sphereRadius_ - distances[indicesInLengthOrder[0]] * distances[indicesInLengthOrder[0]] -
                    distances[indicesInLengthOrder[1]] * distances[indicesInLengthOrder[1]]);

      numSpheres[indicesInLengthOrder[0]] = numSpheres[indicesInLengthOrder[1]] = 1;
      numSpheres[indicesInLengthOrder[2]] = std::ceil(sides[indicesInLengthOrder[2]] / (2 * distances[indicesInLengthOrder[2]]));

      // Re-calculate the distances
      distances[2] = std::max(sides[indicesInLengthOrder[2]] / (2 * numSpheres[indicesInLengthOrder[2]]), sphereRadius_ - maxExtrusion_);

    } else {
      std::cerr << "Case 2-2: One sphere for the shortest side & Multiple spheres for the medium side\n";
      distances[indicesInLengthOrder[0]] = sides[indicesInLengthOrder[0]] / 2;
      distances[indicesInLengthOrder[1]] = distances[indicesInLengthOrder[2]] = dist;

      numSpheres[indicesInLengthOrder[0]] = 1;
      numSpheres[indicesInLengthOrder[1]] = std::ceil(sides[indicesInLengthOrder[1]] / (2 * distances[indicesInLengthOrder[1]]));
      numSpheres[indicesInLengthOrder[2]] = std::ceil(sides[indicesInLengthOrder[2]] / (2 * distances[indicesInLengthOrder[2]]));

      // Re-calculate the distances
      distances[indicesInLengthOrder[1]] = distances[indicesInLengthOrder[2]] =
          std::max(sides[indicesInLengthOrder[1]] / (2 * numSpheres[indicesInLengthOrder[1]]),
                   sides[indicesInLengthOrder[2]] / (2 * numSpheres[indicesInLengthOrder[2]]));
    }
  } else {
    std::cerr << "Case 3: Multiple spheres along all sides\n";
    distances = (sphereRadius_ - maxExtrusion_) * vector_t::Ones(3);

    numSpheres[indicesInLengthOrder[0]] = std::ceil(sides[indicesInLengthOrder[0]] / (2 * distances[indicesInLengthOrder[0]]));
    numSpheres[indicesInLengthOrder[1]] = std::ceil(sides[indicesInLengthOrder[1]] / (2 * distances[indicesInLengthOrder[1]]));
    numSpheres[indicesInLengthOrder[2]] = std::ceil(sides[indicesInLengthOrder[2]] / (2 * distances[indicesInLengthOrder[2]]));

    // Re-calculate the distances
    distances = std::max({sides[indicesInLengthOrder[0]] / (2 * numSpheres[indicesInLengthOrder[0]]),
                          sides[indicesInLengthOrder[1]] / (2 * numSpheres[indicesInLengthOrder[1]]),
                          sides[indicesInLengthOrder[2]] / (2 * numSpheres[indicesInLengthOrder[2]])}) *
                vector_t::Ones(3);
  }
  // re-calculate the sphere radius
  sphereRadius_ = distances.norm();

  sphereCentersInObjectFrame_.resize(numSpheres[0] * numSpheres[1] * numSpheres[2]);
  sphereCentersInWorldFrame_.resize(numSpheres[0] * numSpheres[1] * numSpheres[2]);

  // Sphere spacings along x, y, z
  vector_t spacings(3);
  for (size_t i = 0; i < numSpheres.size(); i++) {
    if (numSpheres[i] > 1) {
      spacings[i] = (sides[i] - 2 * distances[i]) / (numSpheres[i] - 1);
    }
  }

  for (size_t i = 0; i < numSpheres[0]; i++) {
    for (size_t j = 0; j < numSpheres[1]; j++) {
      for (size_t k = 0; k < numSpheres[2]; k++) {
        size_t count = i * numSpheres[1] * numSpheres[2] + j * numSpheres[2] + k;
        sphereCentersInObjectFrame_[count] = vector_t::Zero(3);
        sphereCentersInObjectFrame_[count] << distances[0] + i * spacings[0] - sides[0] / 2, distances[1] + j * spacings[1] - sides[1] / 2,
            distances[2] + k * spacings[2] - sides[2] / 2;
      }
    }
  }
}

void SphereApproximation::approximateCylinder(const scalar_t& radius, const scalar_t& length) {}

}  // namespace ocs2
