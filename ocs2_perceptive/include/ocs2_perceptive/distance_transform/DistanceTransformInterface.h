/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/Types.h>

namespace ocs2 {

/** Distance field which defines a 3D point distance to the "nearest" surface and the gradient of the field. */
class DistanceTransformInterface {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;

  DistanceTransformInterface() = default;
  virtual ~DistanceTransformInterface() = default;

  DistanceTransformInterface(const DistanceTransformInterface&) = delete;
  DistanceTransformInterface& operator=(const DistanceTransformInterface&) = delete;
  DistanceTransformInterface(DistanceTransformInterface&&) = default;
  DistanceTransformInterface& operator=(DistanceTransformInterface&&) = default;

  /** Gets the distance to the given point. */
  virtual scalar_t getValue(const vector3_t& p) const = 0;

  /** Projects the given point to the nearest point on the surface. */
  virtual vector3_t getProjectedPoint(const vector3_t& p) const = 0;

  /** Gets the distance's value and its gradient at the given point. */
  virtual std::pair<scalar_t, vector3_t> getLinearApproximation(const vector3_t& p) const = 0;
};

/** Identity distance transform with constant zero value and zero gradients. */
class IdentityDistanceTransform : public DistanceTransformInterface {
 public:
  IdentityDistanceTransform() = default;
  ~IdentityDistanceTransform() override = default;

  scalar_t getValue(const vector3_t&) const override { return 0.0; }
  vector3_t getProjectedPoint(const vector3_t& p) const override { return p; }
  std::pair<scalar_t, vector3_t> getLinearApproximation(const vector3_t&) const override { return {0.0, vector3_t::Zero()}; }
};

}  // namespace ocs2
