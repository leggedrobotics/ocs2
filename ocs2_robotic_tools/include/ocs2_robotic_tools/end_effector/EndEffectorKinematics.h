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
#include <utility>

#include <ocs2_core/Types.h>

namespace ocs2 {

/** The Kinematics function which maps state-input pair to the end-effector (position, velocity, orientation error) */
template <typename SCALAR_T>
class EndEffectorKinematics {
 public:
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3x_t = Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  using quaternion_t = Eigen::Quaternion<SCALAR_T>;

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
  virtual std::vector<vector3_t> getPosition(const vector_t& state) const = 0;

  /**
   * Get end-effector velocity vectors in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity vectors
   */
  virtual std::vector<vector3_t> getVelocity(const vector_t& state, const vector_t& input) const = 0;

  /**
   * Get orientation error in world frame
   *
   * @note: To calculate the error use quaternionDistance() from ocs2_robotic_tools/common/RotationTransforms.h
   *
   * @param [in] state vector
   * @param [in] referenceOrientation: reference quaternion
   * @return array of orientation errors
   */
  virtual std::vector<vector3_t> getOrientationError(const vector_t& state,
                                                     const std::vector<quaternion_t>& referenceOrientations) const = 0;

  /**
   * Get end-effector position linear approximation in world frame
   *
   * @param [in] state: state vector
   * @return array of position function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& state) const = 0;

  /**
   * Get end-effector velocity linear approximation in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& state,
                                                                                        const vector_t& input) const = 0;

  /**
   * Get end-effector orintation error linear approximation in world frame
   *
   * @note: To calculate the error and Jacobian use quaternionDistance() and quaternionDistanceJacobian() from
   *        ocs2_robotic_tools/common/RotationTransforms.h
   *
   * @param [in] state: state vector
   * @param [in] referenceOrientation: reference quaternion
   * @return array of orientation error linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const = 0;

 protected:
  EndEffectorKinematics(const EndEffectorKinematics&) = default;
};

}  // namespace ocs2
