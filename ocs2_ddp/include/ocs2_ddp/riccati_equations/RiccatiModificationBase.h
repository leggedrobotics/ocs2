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

#include <ocs2_core/Dimensions.h>

namespace ocs2 {

/**
 * The struct contains Riccati equation modification terms.
 */
struct RiccatiModificationBase {
  using array_t = std::vector<RiccatiModificationBase, Eigen::aligned_allocator<RiccatiModificationBase>>;
  using array2_t = std::vector<array_t, Eigen::aligned_allocator<array_t>>;

  using scalar_t = typename Dimensions<0, 0>::scalar_t;
  using dynamic_vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
  using dynamic_matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

  scalar_t time_;

  dynamic_matrix_t deltaQm_;
  dynamic_matrix_t deltaRm_;
  dynamic_matrix_t deltaPm_;

  /** The right pseudo-inverse of Dm */
  dynamic_matrix_t DmDagger_;

  /** Hm is the 2nd derivative of Hamiltonian w.r.t. input. HmInverseConstrained is (I - DmDagerDm)^T inv(Hm) (I - DmDagerDm) */
  dynamic_matrix_t HmInverseConstrained_;
  /** HmInverseConstrainedUUT is VVT decomposition of (I - DmDagerDm) inv(Hm) with dimension n_u*(n_u-n_c)  */
  dynamic_matrix_t HmInverseConstrainedLowRank_;

  /**
   * Displays all variables
   */
  virtual void display() {
    std::cerr << std::endl;
    std::cerr << "time: " << time_ << "\n";
    std::cerr << "deltaQm:\n" << deltaQm_ << "\n";
    std::cerr << "deltaRm:\n" << deltaRm_ << "\n";
    std::cerr << "deltaPm:\n" << deltaPm_ << "\n";

    std::cerr << "DmDagger:\n" << DmDagger_ << "\n";

    std::cerr << "HmInverseConstrained:\n" << HmInverseConstrained_ << "\n";
    std::cerr << "HmInverseConstrainedLowRank:\n" << HmInverseConstrainedLowRank_ << "\n";
    std::cerr << std::endl;
  }
};

}  // namespace ocs2
