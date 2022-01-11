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

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>

namespace ocs2 {

/**
 * Sphere Kinematics implementation using pinocchio.
 *
 * This class uses caching of computation done on the pinocchio::Data in PinocchioInterface.
 * See in the method documentation which pinocchio functions are required to update pinocchio::Data.
 *
 * Example:
 * \code{.cpp}
 *   PinocchioSphereKinematics kinematics(pinocchioSphereInterface, mapping);
 *   pinocchio::forwardKinematics(pinocchioInterface.getModel(), pinocchioInterface.getData(), q);
 *   pinocchio::updateFramePlacements(pinocchioInterface.getModel(), pinocchioInterface.getData());
 *   kinematics.setPinocchioInterface(pinocchioInterface);
 *   const auto pos = kinematics.getPosition(x);
 * \endcode
 */
class PinocchioSphereKinematics final : public EndEffectorKinematics<scalar_t> {
 public:
  using EndEffectorKinematics<scalar_t>::vector3_t;
  using EndEffectorKinematics<scalar_t>::matrix3x_t;
  using EndEffectorKinematics<scalar_t>::quaternion_t;

  /** Constructor
   * @param [in] pinocchioSphereInterface: pinocchio sphere interface.
   * @param [in] mapping: mapping from OCS2 to pinocchio state.
   */
  PinocchioSphereKinematics(PinocchioSphereInterface pinocchioSphereInterface, const PinocchioStateInputMapping<scalar_t>& mapping);

  ~PinocchioSphereKinematics() override = default;
  PinocchioSphereKinematics* clone() const override;
  PinocchioSphereKinematics& operator=(const PinocchioSphereKinematics&) = delete;

  /** Set the pinocchio interface for caching.
   * @note The pinocchio interface must be set before calling the getters.
   * @param [in] pinocchioInterface: pinocchio interface on which computations are expected. It will keep a pointer for the getters.
   */
  void setPinocchioInterface(const PinocchioInterface& pinocchioInterface) {
    pinocchioInterfacePtr_ = &pinocchioInterface;
    mappingPtr_->setPinocchioInterface(pinocchioInterface);
  }

  /** Get the pinocchio sphere interface **/
  const PinocchioSphereInterface& getPinocchioSphereInterface() const { return pinocchioSphereInterface_; };

  /** Get IDs (names) of the links which the spheres approximate */
  const std::vector<std::string>& getIds() const override { return linkIds_; };

  /** Get the sphere center position vectors.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   */
  std::vector<vector3_t> getPosition(const vector_t& state) const override;

  /** Get the sphere center velocity vectors.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q, v)
   */
  std::vector<vector3_t> getVelocity(const vector_t& state, const vector_t& input) const override {
    throw std::runtime_error("[PinocchioSphereKinematics] getVelocity() is not implemented");
  };

  /** Get the sphere center orientation error.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   */
  std::vector<vector3_t> getOrientationError(const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override {
    throw std::runtime_error("[PinocchioSphereKinematics] getOrientationError() is not implemented");
  };

  /** Get the sphere center position linear approximation.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   *       pinocchio::computeJointJacobians(model, data)
   */
  std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& state) const override;

  /** Get the sphere center velocity linear approximation
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a)
   */
  std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& state,
                                                                                const vector_t& input) const override {
    throw std::runtime_error("[PinocchioSphereKinematics] getVelocityLinearApproximation() is not implemented");
  };

  /** Get the sphere center orientation error linear approximation.
   * @note requires pinocchioInterface to be updated with:
   *       pinocchio::forwardKinematics(model, data, q)
   *       pinocchio::updateFramePlacements(model, data)
   *       pinocchio::computeJointJacobians(model, data)
   */
  std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override {
    throw std::runtime_error("[PinocchioSphereKinematics] getOrientationErrorLinearApproximation() is not implemented");
  };

 private:
  PinocchioSphereKinematics(const PinocchioSphereKinematics& rhs);

  const PinocchioInterface* pinocchioInterfacePtr_;
  std::unique_ptr<PinocchioStateInputMapping<scalar_t>> mappingPtr_;
  PinocchioSphereInterface pinocchioSphereInterface_;
  std::vector<std::string> linkIds_;
};

}  // namespace ocs2
