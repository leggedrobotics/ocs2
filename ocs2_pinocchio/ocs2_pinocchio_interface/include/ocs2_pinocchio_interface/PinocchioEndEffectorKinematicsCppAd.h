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

#include <functional>
#include <string>
#include <vector>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace ocs2 {

/**
 * This class provides the CppAD implementation the end-effector Kinematics based on pinocchio. No pre-computation
 * is required. The class has two constructors. The constructor with an additional argument, "updateCallback", is meant
 * for cases where PinocchioStateInputMapping requires some extra update calls on PinocchioInterface, such as the
 * centroidal model mapping (refer to CentroidalModelPinocchioMapping).
 *
 * See also PinocchioEndEffectorKinematics, which uses analytical computation and caching.
 */
class PinocchioEndEffectorKinematicsCppAd final : public EndEffectorKinematics<scalar_t> {
 public:
  using EndEffectorKinematics<scalar_t>::vector3_t;
  using EndEffectorKinematics<scalar_t>::matrix3x_t;
  using EndEffectorKinematics<scalar_t>::quaternion_t;
  using update_pinocchio_interface_callback =
      std::function<void(const ad_vector_t& state, PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface)>;

  /** Constructor
   * @param [in] pinocchioInterface pinocchio interface.
   * @param [in] mapping mapping from OCS2 to pinocchio state.
   * @param [in] endEffectorIds array of end effector names.
   * @param [in] stateDim : size of state vector
   * @param [in] inputDim : size of input vector
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose : print information.
   */
  PinocchioEndEffectorKinematicsCppAd(const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                      std::vector<std::string> endEffectorIds, size_t stateDim, size_t inputDim,
                                      const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2",
                                      bool recompileLibraries = true, bool verbose = false);

  /** Constructor
   * @param [in] pinocchioInterface pinocchio interface.
   * @param [in] mapping mapping from OCS2 to pinocchio state.
   * @param [in] endEffectorIds array of end effector names.
   * @param [in] stateDim : size of state vector
   * @param [in] inputDim : size of input vector
   * @param [in] updateCallback : In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
   *                              use this callback.
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose : print information.
   */
  PinocchioEndEffectorKinematicsCppAd(const PinocchioInterface& pinocchioInterface, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                      std::vector<std::string> endEffectorIds, size_t stateDim, size_t inputDim,
                                      update_pinocchio_interface_callback updateCallback, const std::string& modelName,
                                      const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = false);

  ~PinocchioEndEffectorKinematicsCppAd() override = default;
  PinocchioEndEffectorKinematicsCppAd* clone() const override;
  PinocchioEndEffectorKinematicsCppAd& operator=(const PinocchioEndEffectorKinematicsCppAd&) = delete;

  const std::vector<std::string>& getIds() const override;

  std::vector<vector3_t> getPosition(const vector_t& state) const override;
  std::vector<vector3_t> getVelocity(const vector_t& state, const vector_t& input) const override;
  std::vector<vector3_t> getOrientationError(const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;

  std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& state) const override;
  std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& state,
                                                                                const vector_t& input) const override;
  std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;

 private:
  PinocchioEndEffectorKinematicsCppAd(const PinocchioEndEffectorKinematicsCppAd& rhs);

  ad_vector_t getPositionCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const ad_vector_t& state);
  ad_vector_t getVelocityCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd, const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                               const ad_vector_t& state, const ad_vector_t& input);
  ad_vector_t getOrientationErrorCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
                                       const PinocchioStateInputMapping<ad_scalar_t>& mapping, const ad_vector_t& state,
                                       const ad_vector_t& params);

  std::unique_ptr<CppAdInterface> positionCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> velocityCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> orientationErrorCppAdInterfacePtr_;

  const std::vector<std::string> endEffectorIds_;
  std::vector<size_t> endEffectorFrameIds_;
};

}  // namespace ocs2
