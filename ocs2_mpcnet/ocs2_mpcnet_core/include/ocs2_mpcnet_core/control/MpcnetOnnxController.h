/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

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

#include <onnxruntime/onnxruntime_cxx_api.h>

#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include "ocs2_mpcnet_core/MpcnetDefinitionBase.h"
#include "ocs2_mpcnet_core/control/MpcnetControllerBase.h"

namespace ocs2 {
namespace mpcnet {

/**
 * Convenience function for creating an environment for ONNX Runtime and getting a corresponding shared pointer.
 * @note Only one environment per process can be created. The environment offers some threading and logging options.
 * @return Pointer to the environment for ONNX Runtime.
 */
inline std::shared_ptr<Ort::Env> createOnnxEnvironment() {
  return std::make_shared<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "MpcnetOnnxController");
}

/**
 * A neural network controller using ONNX Runtime based on the Open Neural Network Exchange (ONNX) format.
 * The model of the policy computes u = model(t, x) with
 * t: generalized time (1 x dimensionOfTime),
 * x: relative state (1 x dimensionOfState),
 * u: predicted input (1 x dimensionOfInput),
 * @note The additional first dimension with size 1 for the variables of the model comes from batch processing during training.
 */
class MpcnetOnnxController final : public MpcnetControllerBase {
 public:
  /**
   * Constructor.
   * @note The class is not fully instantiated until calling loadPolicyModel().
   * @param [in] mpcnetDefinitionPtr : Pointer to the MPC-Net definitions.
   * @param [in] referenceManagerPtr : Pointer to the reference manager.
   * @param [in] onnxEnvironmentPtr : Pointer to the environment for ONNX Runtime.
   */
  MpcnetOnnxController(std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr,
                       std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr, std::shared_ptr<Ort::Env> onnxEnvironmentPtr)
      : mpcnetDefinitionPtr_(std::move(mpcnetDefinitionPtr)),
        referenceManagerPtr_(std::move(referenceManagerPtr)),
        onnxEnvironmentPtr_(std::move(onnxEnvironmentPtr)) {}

  ~MpcnetOnnxController() override = default;
  MpcnetOnnxController* clone() const override { return new MpcnetOnnxController(*this); }

  void loadPolicyModel(const std::string& policyFilePath) override;

  vector_t computeInput(const scalar_t t, const vector_t& x) override;
  ControllerType getType() const override { return ControllerType::ONNX; }

  int size() const override { throw std::runtime_error("[MpcnetOnnxController::size] not implemented."); }
  void clear() override { throw std::runtime_error("[MpcnetOnnxController::clear] not implemented."); }
  bool empty() const override { throw std::runtime_error("[MpcnetOnnxController::empty] not implemented."); }
  void concatenate(const ControllerBase* otherController, int index, int length) override {
    throw std::runtime_error("[MpcnetOnnxController::concatenate] not implemented.");
  }

 private:
  using tensor_element_t = float;

  MpcnetOnnxController(const MpcnetOnnxController& other)
      : MpcnetOnnxController(other.mpcnetDefinitionPtr_, other.referenceManagerPtr_, other.onnxEnvironmentPtr_) {
    if (!other.policyFilePath_.empty()) {
      loadPolicyModel(other.policyFilePath_);
    }
  }

  std::shared_ptr<MpcnetDefinitionBase> mpcnetDefinitionPtr_;
  std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;
  std::shared_ptr<Ort::Env> onnxEnvironmentPtr_;
  std::string policyFilePath_;
  std::unique_ptr<Ort::Session> sessionPtr_;
  std::vector<const char*> inputNames_;
  std::vector<const char*> outputNames_;
  std::vector<std::vector<int64_t>> inputShapes_;
  std::vector<std::vector<int64_t>> outputShapes_;
};

}  // namespace mpcnet
}  // namespace ocs2
