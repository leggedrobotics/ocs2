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

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace ocs2 {

/**
 * End-effector Kinematics implementation using pinocchio and CppAD.
 *
 * @note See also PinocchioSphereKinematics, which uses analytical computation and caching.
 */
class PinocchioSphereKinematicsCppAd final : public EndEffectorKinematics<scalar_t> {
 public:
  using EndEffectorKinematics<scalar_t>::vector3_t;
  using EndEffectorKinematics<scalar_t>::matrix3x_t;
  using EndEffectorKinematics<scalar_t>::quaternion_t;

  struct SphereApproxParam {
    SphereApproxParam(vector3_t placementTranslation, quaternion_t placementOrientation, std::vector<vector3_t> sphereCentersToObjectCenter)
        : placementTranslation(std::move(placementTranslation)),
          placementOrientation(std::move(placementOrientation)),
          sphereCentersToObjectCenter(std::move(sphereCentersToObjectCenter)) {}
    vector3_t placementTranslation;
    quaternion_t placementOrientation;
    std::vector<vector3_t> sphereCentersToObjectCenter;
  };

  /** Constructor
   * @param [in] pinocchioInterface pinocchio interface.
   * @param [in] pinocchioSphereInterface pinocchio sphere interface
   * @param [in] mapping mapping from OCS2 to pinocchio state.
   * @param [in] stateDim : size of state vector
   * @param [in] inputDim : size of input vector
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose : print information.
   */
  PinocchioSphereKinematicsCppAd(const PinocchioInterface& pinocchioInterface, PinocchioSphereInterface pinocchioSphereInterface,
                                 const PinocchioStateInputMapping<ad_scalar_t>& mapping, size_t stateDim, size_t inputDim,
                                 const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                                 bool verbose = false);

  ~PinocchioSphereKinematicsCppAd() override = default;
  PinocchioSphereKinematicsCppAd* clone() const override;
  PinocchioSphereKinematicsCppAd& operator=(const PinocchioSphereKinematicsCppAd&) = delete;

  const PinocchioSphereInterface& getPinocchioSphereInterface() const { return pinocchioSphereInterface_; };

  const std::vector<std::string>& getIds() const override { return linkIds_; };

  std::vector<vector3_t> getPosition(const vector_t& state) const override;
  std::vector<vector3_t> getVelocity(const vector_t& state, const vector_t& input) const override {
    throw std::runtime_error("[PinocchioSphereKinematicsCppAd] getVelocity() is not implemented");
  };
  std::vector<vector3_t> getOrientationError(const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override {
    throw std::runtime_error("[PinocchioSphereKinematicsCppAd] getOrientationError() is not implemented");
  };

  std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& state) const override;
  std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& state,
                                                                                const vector_t& input) const override {
    throw std::runtime_error("[PinocchioSphereKinematicsCppAd] getVelocityLinearApproximation() is not implemented");
  };
  std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override {
    throw std::runtime_error("[PinocchioSphereKinematicsCppAd] getOrientationErrorLinearApproximation() is not implemented");
  };

 private:
  PinocchioSphereKinematicsCppAd(const PinocchioSphereKinematicsCppAd& rhs);

  std::vector<SphereApproxParam> createSphereApproxParams() const;
  ad_vector_t getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const std::vector<SphereApproxParam>& sphereApproxParams, const ad_vector_t& state);

  std::unique_ptr<CppAdInterface> positionCppAdInterfacePtr_;

  PinocchioSphereInterface pinocchioSphereInterface_;
  std::vector<std::string> linkIds_;
};

}  // namespace ocs2
