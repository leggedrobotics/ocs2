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

#include <algorithm>
#include <cmath>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SphereApproximation::SphereApproximation(const hpp::fcl::CollisionGeometry& geometry, size_t geomObjId, scalar_t maxExcess,
                                         scalar_t shrinkRatio)
    : geomObjId_(geomObjId), maxExcess_(maxExcess), shrinkRatio_(shrinkRatio) {
  if (shrinkRatio <= 0.0 || shrinkRatio >= 1.0) {
    throw std::runtime_error("[SphereApproximation] shrinkRation must be larger than 0.0 and smaller than 1.0!");
  }
  const auto& nodeType = geometry.getNodeType();
  switch (nodeType) {
    case hpp::fcl::NODE_TYPE::GEOM_BOX: {
      const auto* boxPtr = dynamic_cast<const hpp::fcl::Box*>(&geometry);
      approximateBox(boxPtr->halfSide * 2);
      break;
    }
    case hpp::fcl::NODE_TYPE::GEOM_CYLINDER: {
      const auto* cylinderPtr = dynamic_cast<const hpp::fcl::Cylinder*>(&geometry);
      approximateCylinder(cylinderPtr->radius, cylinderPtr->halfLength * 2);
      break;
    }
    case hpp::fcl::NODE_TYPE::GEOM_SPHERE: {
      const auto* spherePtr = dynamic_cast<const hpp::fcl::Sphere*>(&geometry);
      numSpheres_ = 1;
      sphereRadius_ = spherePtr->radius;
      sphereCentersToObjectCenter_.resize(1);
      sphereCentersToObjectCenter_[0] = vector_t::Zero(3);
      break;
    }
    default:
      throw std::runtime_error("[SphereApproximation] Undefined shape primitive for sphere approximation");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SphereApproximation::approximateBox(const vector_t& sides) {
  // indices of the shortest, medium, and longest side
  size_array_t idxSorted = {0, 1, 2};
  std::stable_sort(idxSorted.begin(), idxSorted.end(),
                   [&sides](const size_t& idx1, const size_t& idx2) { return sides[idx1] < sides[idx2]; });

  vector_t initSphereRadii(3);
  size_t caseIdx;
  initSphereRadii << sides.norm() / 2, sides[idxSorted[0]] / 2 + maxExcess_, std::sqrt(3) * maxExcess_ / (std::sqrt(3) - 1);
  sphereRadius_ = initSphereRadii.minCoeff(&caseIdx);

  // Distance between the first sphere center and the corner along x, y, z
  vector_t distances(3);
  // Number of spheres along x, y, z
  vector_t numSpheres(3);

  switch (caseIdx) {
    case 0:
      distances = sides / 2;
      numSpheres = vector_t::Ones(3);

      // No re-calculation of the distances is required
      break;
    case 1: {
      scalar_t dist = std::sqrt(std::pow(sphereRadius_, 2) - std::pow(sides[idxSorted[0]] / 2, 2)) / std::sqrt(2);

      if (dist >= sides[idxSorted[1]] / 2) {
        distances[idxSorted[0]] = sides[idxSorted[0]] / 2;
        distances[idxSorted[1]] = sides[idxSorted[1]] / 2;
        distances[idxSorted[2]] =
            std::sqrt(std::pow(sphereRadius_, 2) - std::pow(distances[idxSorted[0]], 2) - std::pow(distances[idxSorted[1]], 2));

        numSpheres[idxSorted[0]] = numSpheres[idxSorted[1]] = 1;
        numSpheres[idxSorted[2]] = std::ceil(sides[idxSorted[2]] / (2 * distances[idxSorted[2]]));

        // Re-calculate the distances
        distances[idxSorted[2]] = std::max(sides[idxSorted[2]] / (2 * numSpheres[idxSorted[2]]), sphereRadius_ - maxExcess_);

      } else {
        distances[idxSorted[0]] = sides[idxSorted[0]] / 2;
        distances[idxSorted[1]] = distances[idxSorted[2]] = dist;

        numSpheres[idxSorted[0]] = 1;
        numSpheres[idxSorted[1]] = std::ceil(sides[idxSorted[1]] / (2 * distances[idxSorted[1]]));
        numSpheres[idxSorted[2]] = std::ceil(sides[idxSorted[2]] / (2 * distances[idxSorted[2]]));

        // Re-calculate the distances
        distances[idxSorted[1]] = distances[idxSorted[2]] =
            std::max(sides[idxSorted[1]] / (2 * numSpheres[idxSorted[1]]), sides[idxSorted[2]] / (2 * numSpheres[idxSorted[2]]));
      }
      break;
    }
    case 2:
      distances = (sphereRadius_ - maxExcess_) * vector_t::Ones(3);

      numSpheres[idxSorted[0]] = std::ceil(sides[idxSorted[0]] / (2 * distances[idxSorted[0]]));
      numSpheres[idxSorted[1]] = std::ceil(sides[idxSorted[1]] / (2 * distances[idxSorted[1]]));
      numSpheres[idxSorted[2]] = std::ceil(sides[idxSorted[2]] / (2 * distances[idxSorted[2]]));

      // Re-calculate the distances
      distances = std::max({sides[idxSorted[0]] / (2 * numSpheres[idxSorted[0]]), sides[idxSorted[1]] / (2 * numSpheres[idxSorted[1]]),
                            sides[idxSorted[2]] / (2 * numSpheres[idxSorted[2]])}) *
                  vector_t::Ones(3);
      break;
  }

  // re-calculate the sphere radius
  sphereRadius_ = distances.norm();

  numSpheres_ = numSpheres[0] * numSpheres[1] * numSpheres[2];
  sphereCentersToObjectCenter_.resize(numSpheres_);

  // Sphere spacings along x, y, z
  vector_t spacings(3);
  for (size_t i = 0; i < numSpheres.size(); i++) {
    if (numSpheres[i] > 1) {
      spacings[i] = (sides[i] - 2 * distances[i]) / (numSpheres[i] - 1);
    }
  }

  size_t count = 0;
  for (size_t i = 0; i < numSpheres[0]; i++) {
    for (size_t j = 0; j < numSpheres[1]; j++) {
      for (size_t k = 0; k < numSpheres[2]; k++) {
        sphereCentersToObjectCenter_[count] << distances[0] + i * spacings[0] - sides[0] / 2, distances[1] + j * spacings[1] - sides[1] / 2,
            distances[2] + k * spacings[2] - sides[2] / 2;
        count++;
      }
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SphereApproximation::approximateCylinder(const scalar_t radius, const scalar_t length) {
  // First, approximate the rectangle cross-section of the cylinder
  vector_t sides(2);
  sides << 2 * radius, length;

  size_array_t idxSorted = {0, 1};
  if (sides(0) > sides(1)) {
    std::swap(idxSorted[0], idxSorted[1]);
  }

  scalar_t maxExcessL = maxExcess_ * shrinkRatio_;
  vector_t distances(2);
  vector_t numSpheres(2);

  approximateRectanglularCrossSection(sides, idxSorted, maxExcess_, sphereRadius_, numSpheres, distances);
  bool recursiveApproximation = false;  // If recursive approximation of the cylinder base is necessary.
  if (numSpheres[0] > 1) {
    // More than one sphere is required along radial direction with maxExcess. Re-approximate the rectangular cross-section using the
    // reduced threshold in favor of the later approximation of the circular base.
    approximateRectanglularCrossSection(sides, idxSorted, maxExcessL, sphereRadius_, numSpheres, distances);
    recursiveApproximation = true;
  }

  scalar_t spacingLength;
  if (numSpheres[1] > 1) {
    spacingLength = (sides[1] - 2 * distances[1]) / (numSpheres[1] - 1);
  }

  // Second, approximate the circle base of the cylinder
  vector_array_t circleCentersToBaseCenter;
  if (!recursiveApproximation) {
    circleCentersToBaseCenter.push_back(vector_t::Zero(2));
  } else {
    scalar_t radiusBase = radius;
    scalar_t radiusCircle = std::sqrt(std::pow(sphereRadius_, 2) - std::pow(distances[1], 2));
    scalar_t maxExcessR = maxExcess_ - maxExcessL;

    while (recursiveApproximation) {
      scalar_t shift, alpha, numCircles;
      recursiveApproximation = approximateCircleBase(radiusBase, radiusCircle, maxExcessR, shift, alpha, numCircles);

      for (size_t i = 0; i < numCircles; i++) {
        vector_t circleCenter(2);
        circleCenter << shift * std::sin(i * alpha), shift * std::cos(i * alpha);
        circleCentersToBaseCenter.push_back(circleCenter);
      }

      // Enclose the uncovered area by another circle with radiusBase
      radiusBase = shift / std::cos(alpha / 2) - radiusCircle;
    }
  }

  numSpheres[0] = circleCentersToBaseCenter.size();
  numSpheres_ = numSpheres[0] * numSpheres[1];
  sphereCentersToObjectCenter_.resize(numSpheres_);

  size_t count = 0;
  for (size_t i = 0; i < numSpheres[1]; i++) {
    for (size_t j = 0; j < numSpheres[0]; j++) {
      sphereCentersToObjectCenter_[count] << circleCentersToBaseCenter[j], distances[1] + i * spacingLength - sides[1] / 2;
      count++;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SphereApproximation::approximateRectanglularCrossSection(const vector_t& sides, const size_array_t& idxSorted,
                                                              const scalar_t maxExcess, scalar_t& sphereRadius, vector_t& numSpheres,
                                                              vector_t& distances) {
  vector_t initSphereRadii(3);
  size_t caseIdx;
  initSphereRadii << sides.norm() / 2, sides[idxSorted[0]] / 2 + maxExcess, std::sqrt(2) * maxExcess / (std::sqrt(2) - 1);
  sphereRadius = initSphereRadii.minCoeff(&caseIdx);

  switch (caseIdx) {
    case 0:
      distances = sides / 2;
      numSpheres = vector_t::Ones(2);
      break;
    case 1:
      distances[idxSorted[0]] = sides[idxSorted[0]] / 2;
      distances[idxSorted[1]] = std::sqrt(std::pow(sphereRadius, 2) - std::pow(distances[idxSorted[0]], 2));

      numSpheres[idxSorted[0]] = 1;
      numSpheres[idxSorted[1]] = std::ceil(sides[idxSorted[1]] / (2 * distances[idxSorted[1]]));

      // Re-calculate the distances
      distances[idxSorted[1]] = sides[idxSorted[1]] / (2 * numSpheres[idxSorted[1]]);
      break;
    case 2:
      distances = (sphereRadius - maxExcess) * vector_t::Ones(2);

      numSpheres[idxSorted[0]] = std::ceil(sides[idxSorted[0]] / (2 * distances[idxSorted[0]]));
      numSpheres[idxSorted[1]] = std::ceil(sides[idxSorted[1]] / (2 * distances[idxSorted[1]]));

      // Re-calculate the distances
      distances = std::max(sides[idxSorted[0]] / (2 * numSpheres[idxSorted[0]]), sides[idxSorted[1]] / (2 * numSpheres[idxSorted[1]])) *
                  vector_t::Ones(2);
      break;
  }
  sphereRadius = distances.norm();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool SphereApproximation::approximateCircleBase(const scalar_t radiusBase, const scalar_t radiusSphereCrossSection,
                                                const scalar_t maxExcessR, scalar_t& shift, scalar_t& alpha, scalar_t& numCircles) {
  if (radiusSphereCrossSection < radiusBase - std::numeric_limits<scalar_t>::epsilon()) {
    shift = radiusBase + std::min(0.0, maxExcessR - radiusSphereCrossSection);
    alpha =
        2 * std::acos((std::pow(radiusBase, 2) + std::pow(shift, 2) - std::pow(radiusSphereCrossSection, 2)) / (2 * radiusBase * shift));

    numCircles = std::ceil(2 * M_PI / alpha);

    // Re-calculate alpha & shift
    alpha = 2 * M_PI / numCircles;
    vector_t intersection(2);
    intersection << radiusBase * std::sin(alpha / 2), radiusBase * std::cos(alpha / 2);
    shift = intersection[1] - std::sqrt(std::pow(radiusSphereCrossSection, 2) - std::pow(intersection[0], 2));

    return true;

  } else {
    shift = 0;
    alpha = 0;
    numCircles = 1;

    return false;
  }
}

}  // namespace ocs2
