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
#include <vector>

#include <ocs2_pinocchio_interface/EndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2 {

class PinocchioEndEffectorKinematics final : public EndEffectorKinematics<scalar_t> {
 public:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using matrix3x_t = Eigen::Matrix<scalar_t, 3, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
  using quaternion_t = Eigen::Quaternion<scalar_t>;

  PinocchioEndEffectorKinematics(const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<scalar_t>& mapping,
                                 std::vector<std::string> endEffectorIds);

  ~PinocchioEndEffectorKinematics() override = default;
  PinocchioEndEffectorKinematics* clone() const override;
  PinocchioEndEffectorKinematics& operator=(const PinocchioEndEffectorKinematics&) = delete;

  const std::vector<std::string>& getIds() const override;

  /** Get the end effector position vectors.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   */
  std::vector<vector3_t> getPositions(const vector_t& state) override;

  /** Get the end effector poses.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   */
  std::vector<std::pair<vector3_t, quaternion_t>> getPoses(const vector_t& state) override;

  /** Get the end effector position vectors.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q, v)
   */
  std::vector<vector3_t> getVelocities(const vector_t& state, const vector_t& input) override;

  /** Get the end effector position linear approximation.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   *       pinocchio::computeJointJacobians(model, data)
   */
  std::vector<VectorFunctionLinearApproximation> getPositionsLinearApproximation(const vector_t& state) override;

  /** Get the end effector velocity linear approximation
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a)
   */
  std::vector<VectorFunctionLinearApproximation> getVelocitiesLinearApproximation(const vector_t& state, const vector_t& input) override;

  void setPinocchioInterface(const PinocchioInterface& pinocchioInterface) { pinocchioInterfacePtr_ = &pinocchioInterface; }

 private:
  PinocchioEndEffectorKinematics(const PinocchioEndEffectorKinematics& rhs);

  const PinocchioInterface* pinocchioInterfacePtr_;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
  const std::vector<std::string> endEffectorIds_;
  std::vector<size_t> endEffectorFrameIds_;
};

}  // namespace ocs2
