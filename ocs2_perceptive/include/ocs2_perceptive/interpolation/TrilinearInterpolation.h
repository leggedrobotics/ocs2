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

#include <array>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace trilinear_interpolation {

/**
 * Compute the value of a function at a queried position using tri-linear interpolation on a 3D-grid.
 *
 * @param resolution The resolution of the grid.
 * @param referenceCorner The reference position on the 3-D grid closest to the point.
 * @param cornerValues The values around the reference corner, in the order:
 *  (0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (1, 0, 1), (0, 1, 1), (1, 1, 1).
 * @param position The queried position.
 * @tparam Scalar : The Scalar type.
 * @return Scalar : The interpolated function's value at the queried position.
 */
template <typename Scalar>
Scalar getValue(Scalar resolution, const Eigen::Matrix<Scalar, 3, 1>& referenceCorner, const std::array<Scalar, 8>& cornerValues,
                const Eigen::Matrix<Scalar, 3, 1>& position);

/**
 * Computes a first-order approximation of the function at a queried position using tri-linear interpolation on a 3D-grid.
 *
 * @param resolution The resolution of the grid.
 * @param referenceCorner The reference position on the 3-D grid closest to the point.
 * @param cornerValues The values around the reference corner, in the order:
 *  (0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 0, 1), (1, 0, 1), (0, 1, 1), (1, 1, 1).
 * @param position The queried position.
 * @tparam Scalar : The Scalar type.
 * @return std::pair<Scalar, Eigen::Matrix<Scalar, 3, 1>> : A tuple containing the value and jacobian.
 */
template <typename Scalar>
std::pair<Scalar, Eigen::Matrix<Scalar, 3, 1>> getLinearApproximation(Scalar resolution, const Eigen::Matrix<Scalar, 3, 1>& referenceCorner,
                                                                      const std::array<Scalar, 8>& cornerValues,
                                                                      const Eigen::Matrix<Scalar, 3, 1>& position);

}  // namespace trilinear_interpolation
}  // namespace ocs2

#include "implementation/TrilinearInterpolation.h"
