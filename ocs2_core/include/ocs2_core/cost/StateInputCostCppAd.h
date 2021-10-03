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

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_core/cost/StateInputCost.h>

namespace ocs2 {

/** CppAD state-input cost base class*/
class StateInputCostCppAd : public StateInputCost {
 public:
  StateInputCostCppAd() = default;
  ~StateInputCostCppAd() override = default;

  /** Initialize the CppAd interface
   * @param stateDim : state vector dimension.
   * @param inputDim : state vector dimension.
   * @param parameterDim : parameter vector dimension, set to 0 if getParameters() is not used.
   * @param modelName : Name of the generate model library.
   * @param modelFolder : Folder where the model library files are saved.
   * @param recompileLibraries : If true, always compile the model library, else try to load existing library if available.
   * @param verbose : Print information.
   */
  void initialize(size_t stateDim, size_t inputDim, size_t parameterDim, const std::string& modelName,
                  const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true, bool verbose = true);

  /** Get the parameter vector */
  virtual vector_t getParameters(scalar_t time, const TargetTrajectories& targetTrajectories) const { return vector_t(0); };

  /** Cost evaluation */
  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories,
                    const PreComputation&) const override;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation&) const override;

 protected:
  StateInputCostCppAd(const StateInputCostCppAd& rhs);

  /** The CppAD cost function */
  virtual ad_scalar_t costFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                   const ad_vector_t& parameters) const = 0;

 private:
  std::unique_ptr<ocs2::CppAdInterface> adInterfacePtr_;
};

}  // namespace ocs2
