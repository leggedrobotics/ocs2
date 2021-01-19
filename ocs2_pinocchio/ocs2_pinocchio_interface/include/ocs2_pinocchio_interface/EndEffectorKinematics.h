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

#include <string>

#include <ocs2_core/Types.h>

namespace ocs2 {

/** The Kinematics function which maps state-input pair to the end-effector (position, velocity) */
template <typename SCALAR_T>
class EndEffectorKinematics {
 public:
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3x_t = Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;

  EndEffectorKinematics() = default;
  virtual ~EndEffectorKinematics() = default;
  virtual EndEffectorKinematics* clone() const = 0;
  EndEffectorKinematics& operator=(const EndEffectorKinematics&) = delete;

  /** Get end-effector IDs (names) */
  virtual const std::vector<std::string>& getIds() const = 0;

  /**
   * Get end-effector position vectors in world frame
   *
   * @param [in] state vector
   * @return array of position vectors
   */
  virtual std::vector<vector3_t> getPositions(const vector_t& state) = 0;

  /**
   * Get end-effector velocity vectors in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity vectors
   */
  virtual std::vector<vector3_t> getVelocities(const vector_t& state, const vector_t& input) = 0;

  /**
   * Get end-effector position linear approximation in world frame
   *
   * @param [in] state: state vector
   * @return array of position function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> getPositionsLinearApproximation(const vector_t& state) {
    throw std::runtime_error("[EndEffectorKinematics] getPositionLinearApproximation() not implemented");
  }

  /**
   * Get end-effector velocity linear approximation in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> getVelocitiesLinearApproximation(const vector_t& state, const vector_t& input) {
    throw std::runtime_error("[EndEffectorKinematics] getVelocityLinearApproximation() not implemented");
  }

 protected:
  EndEffectorKinematics(const EndEffectorKinematics&) = default;
};

}  // namespace ocs2
