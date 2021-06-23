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

#include "ocs2_core/Types.h"

#include <cppad/cg.hpp>

namespace ocs2 {

/**
 * Get the inverse of the sub-block of the centroidal momentum matrix which corresponds to the floating base variables.
 *  Ab_inv = [1/m I_{3,3}, -1/m*Ab_12*Ab_22^(-1),
 *               O_{3,3}, Ab_22^(-1)]
 *
 * @param [in] A(q): centroidal momentum matrix
 * @param [out] Ab_inv(q): inverse of the 6x6 left-block of A(q)
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> computeFloatingBaseCentroidalMomentumMatrixInverse(const Eigen::Matrix<SCALAR_T, 6, 6>& Ab) {
  const SCALAR_T mass = Ab(0, 0);
  Eigen::Matrix<SCALAR_T, 3, 3> Ab_22_inv = Ab.template block<3, 3>(3, 3).inverse();
  Eigen::Matrix<SCALAR_T, 6, 6> Ab_inv = Eigen::Matrix<SCALAR_T, 6, 6>::Zero();
  Ab_inv << 1.0 / mass * Eigen::Matrix<SCALAR_T, 3, 3>::Identity(), -1.0 / mass * Ab.template block<3, 3>(0, 3) * Ab_22_inv,
      Eigen::Matrix<SCALAR_T, 3, 3>::Zero(), Ab_22_inv;
  return Ab_inv;
}

}  // namespace ocs2
