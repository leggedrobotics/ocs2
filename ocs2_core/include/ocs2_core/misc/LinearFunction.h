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

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>

namespace ocs2 {

/**
 * Affine function class in the form \f$ u_{ff}(t) + K(t) \f$ where \f$ u_{ff} \f$ is a matrix of size \f$ d_1 * d_2 \f$ and
 * \f$ K \f$ is a matrix of size \f$ d_1 * n_x \f$.
 *
 * @tparam DIM1: \f$ d_1 \f$.
 * @tparam DIM2: \f$ d_2 \f$.
 */
template <size_t STATE_DIM, int DIM1, int DIM2, typename SCALAR = double>
class LinearFunction {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LinearFunction() : time_(0), uff_(0), deltaUff_(0), k_(0) {}

  /**
   * Updates the internal variables with the given input
   * @param [in] arg
   */
  void swap(LinearFunction& arg) {
    time_.swap(arg.time_);
    uff_.swap(arg.uff_);
    deltaUff_.swap(arg.deltaUff_);
    k_.swap(arg.k_);
  }

  /**
   * Sets all the data containers to zero
   */
  void setZero() {
    std::fill(uff_.begin(), uff_.end(), Eigen::Matrix<SCALAR, DIM1, DIM2>::Zero());
    std::fill(deltaUff_.begin(), deltaUff_.end(), Eigen::Matrix<SCALAR, DIM1, DIM2>::Zero());
    std::fill(k_.begin(), k_.end(), Eigen::Matrix<SCALAR, DIM1, STATE_DIM>::Zero());
  }

  /**
   * Clears the internal variables
   */
  void clear() {
    time_.clear();
    uff_.clear();
    deltaUff_.clear();
    k_.clear();
  }

  /**
   * Returns whether the class is empty (i.e. whether its size is 0).
   *
   * @return true if the time container size is 0, false otherwise.
   */
  bool empty() const { return time_.empty(); }

  /**
   * Returns the size of the controller (in particular the time stamp).
   *
   * @return the size of the controller.
   */
  size_t size() const { return time_.size(); }

  std::vector<SCALAR> time_;
  std::vector<Eigen::Matrix<SCALAR, DIM1, DIM2>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, DIM1, DIM2>>> uff_;
  std::vector<Eigen::Matrix<SCALAR, DIM1, DIM2>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, DIM1, DIM2>>> deltaUff_;
  std::vector<Eigen::Matrix<SCALAR, DIM1, STATE_DIM>, Eigen::aligned_allocator<Eigen::Matrix<SCALAR, DIM1, STATE_DIM>>> k_;
};

}  // namespace ocs2
